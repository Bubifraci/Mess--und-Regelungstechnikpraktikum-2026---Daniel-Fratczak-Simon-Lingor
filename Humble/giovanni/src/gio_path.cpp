#include "giovanni/gio_path.h"

using namespace std;

void CGioController::InitDefault(){
  this->loop_exit    = 0;
  this->AXIS_LENGTH  = 0.440;         //Achsenabstand der Räder [m]
  this->Vm           = 1;             //Maximale Bahngeschwindigkeit
  this->d_y          = 0.001;         //Minimaler Abstand zum Zielpunkt, um das Teilen durch 0 zu vermeiden
  this->d_th         = 0.34;          //Minimaler Winkel zwischen Roboterorientierung und globaler x-Achse
  this->kr_max       = 2/AXIS_LENGTH; //Maximale Roboterkrümmung
  this->u0           = Vm/2.0;        //Aktuelle Geschwindigkeit
  this->a            = 1.2;
  this->epsilon      = 1e-4;          //Sehr kleiner Wert
  this->path         = new CCurve();  //Initialisiere Kurve zum Verfolgen (noch leer)
  this->setPose(0.0, 0.0, 0);         //Anfangspose
  this->setLocalSystem(0.0);
}

void  CGioController::setLocalSystem(double ang){
  ex[0] = cos(ang); 
  ex[1] = sin(ang); 
  ey[0] = -sin(ang); 
  ey[1] = cos(ang); 
}

//Wert für h, wenn y = 0
double  CGioController::H_case_1(double y, double theta, double u, double alpha, double *gamma){
  double h_j = SQR(kr_max) / (SQR(theta) * SQR(1+2*alpha));
  *gamma = 2*alpha*u*sqrt(h_j);
  return h_j;
};

//Wert für h, wenn y != 0
double CGioController::H_case_2(double y, double theta, double u, double alpha, double *gamma) {
  double term1 = SQR(theta) / SQR(y);
  double term2 = 4.0 * SQR(kr_max) / (SQR(y) * SQR(1.0 + 2.0 * alpha));
  
  double h_j = 0.5 * (-term1 + sqrt(SQR(term1) + term2));
  
  if (h_j < 0.0 || std::isnan(h_j)) {
      h_j = 1e-6;
  }

  *gamma = 2.0 * alpha * u * sqrt(h_j);
  return h_j;
}

void CGioController::transformPath(double x, double y, double yaw) {
    if (this->path) {
        this->path->transformToPose(x, y, yaw);
    }
}

double  CGioController::Compute_W(double y, double theta, double a, double u, int *err){
  double res, h, gamma;

  //Ist der Fehler y nahe 0 (|y| < epsilon) -> h wird durch erste Formel bestimmt
  if(fabs(y) < epsilon) {
    //Je nachdem, ob theta oder d_th größer ist, wird das als Theta für die H-Werte gegeben -> teilen durch 0 vermeiden
    if(fabs(theta) >= d_th) {
      //Wenn der Aktuelle Winkel größer ist als d_th
     cerr << "H1_1\n";
    h = H_case_1(y, theta, u, a, &gamma);
    *err = 0;
    } else {
      //Wenn der aktuelle Winkel kleiner ist als d_th
      cerr << "H1_2\n";
      h = H_case_1(y, d_th, u, a, &gamma);
      *err = 0;
    }
  } else {
    //Ist der Abstand zum Zielpunkt größer als 0 -> H wird durch zweite Formel bestimmt
    if(fabs(y) >= d_y) {
      //Wenn der Abstand zum Zielpunkt größer ist als d_y -> Rechne normal mit y
      h = H_case_2(y, theta, u, a, &gamma);
      *err = 0;
    } else {
      //Wenn der Abstand zum Zielpunkt kleiner ist als d_y -> Gebe d_y als y, um Teilen durch 0 zu vermeiden
      //Je nachdem, ob theta oder d_th größer ist, wird das als Theta für die H-Werte gegeben -> Teilen durch 0 vermeiden
      if(fabs(theta) < d_th) {
        //Wenn der aktuelle Winkel kleiner ist als d_th
        h = H_case_2(d_y, d_th, u, a, &gamma);
        *err = 0;
      } else {
        //Wenn der aktuelle Winkel größer ist als d_th
        h = H_case_2(d_y, theta, u, a, &gamma);
        *err = 0;
      }
    }
  }

  //Je nachdem ob theta größer ist als d_th, wird w mit sin(theta)/theta oder ohne (Teilen durch 0 -> sin(theta) geht aber sowieso gegen 0, gemäß L'hospital konvergiert es also gegen 1) gerechnet 
  if(fabs(theta) >= d_th) {
    res = -h * u * y * sin(theta)/theta - gamma * theta;
  } else {
    if(fabs(theta) <= d_th && fabs(theta) >= d_th_0){
	 res = -h * u * y - gamma * theta;
    } else {
	 res = -h * u * y;
    }
  }

  return res;
}

CGioController::CGioController(){
  this->InitDefault();
  giofile.open("pos.dat");
};

//Beende den giofile sobald der Code terminiert, und schreibe den Cache in die Datei. Schließe den Path
CGioController::~CGioController(){
  giofile.flush();
  giofile.close();
  giofile.clear();  
  delete path;
}
    
//Schreibe den Betrag des Abstands der Räder  in AXIS_Length und setze Kr_max auf 2/AXIS_LENGTH
void CGioController::setAxisLength(double val) {
  if(fabs(val) > 0) {
    this->AXIS_LENGTH = fabs(val);
    this->kr_max = 2/AXIS_LENGTH;
  }
}

//Gebe Abstand zwischen den Rädern zurück
double CGioController::getAxisLength() {
  return this->AXIS_LENGTH;
}

//Setze Geschwindigkeit (absolut mit abs = 1; oder relativ)
void CGioController::setCurrentVelocity(double val, int abs) {
  if(abs) {
    this->u0 = val;
  } else {
    this->u0 = (this->u0>val)?(val+this->u0):(this->u0+val);
  }
  this->Vm = this->u0 * 2.0;
}

//Gebe die aktuelle Geschwindigkeit zurück
double CGioController::getCurrentVelocity() {
  return this->u0;
}

//Überschreibe die aktuelle Pose
void CGioController::setPose(double x, double y, double phi) {
  this->x0 = x;
  this->y0 = y;
  this->phi0 = phi;
  NormalizeAngle(this->phi0);
}

//Gebe die aktuelle Pose zurück
void CGioController::getPose(double &x, double &y, double &ph) {
  x = this->x0;
  y = this->y0;
  ph = this->phi0;
}

//Fetch alle Path relevanten Parameter aus der .dat Datei des Paths
int CGioController::getPathFromFile(const char* fname){
  int res = path->LoadFromFile(fname);
  if(res){
    path->initTraversal();
    this->loop_exit = path->getNext();
  }
  return res;
}

//In dieser Schleife werden alle Punkte des Paths durchiteriert und geschaut, ob die Distanz realistisch ist -> Kurzum: Kann die Kurve gefahren werden?
int CGioController::canDetermineRobotPosition(int looped){
  int exit;       //Boolean, ob der Pfad realistisch abfahrbar ist (1 für Ja, 0 für Nein)
  int check_prev; //Unbenutzte Variable

  exit       = 0;
  check_prev = 0;

  //Prüfe entweder alle Punkte (loop_exit != 0) oder bis ein Punkt des Pfads noch zu erreichen ist (Distanz > 0.1m)
  while(this->loop_exit != 0 && !exit) {
    if(path->pointInn(x0,y0)) {
      if(path->getDistanceToEnd(x0, y0) > 0.1) { 
        //Ist die Distanz zu dem aktuellen Punkt größer als 0,1 Meter, so ist der Pfad realistisch abfahrbar
        exit = 1;
        //Schreibe ins Giofile den Betrag der Distanz zum aktuellen Zielpunkt
        //giofile << path->getDistanceToEnd(x0, y0) << " "; // u 1
        continue;
      }
    }
    //Prüfe den nächsten Punkt des Pfads
    this->loop_exit = path->getNext(looped);
  }
	
  //Gebe Ergebnis der Pfaderreichbarkeit zurück
  return exit;
}

int CGioController::getNextState(double &u, double &w, double &vleft, double &vright, int looped){
  double l, phic = 0, pathAng = 0, tmpw;
  int err;
  
  if(!canDetermineRobotPosition(looped)) {
    //Ist der Pfad nicht erreichbar/abfahrbar, setze alles auf 0 und breche ab
    u = 0;
    w = 0;
    vleft = 0;
    vright = 0;
    return 0;
  }
  //Ansonsten mache hier weiter
	
  //l ist die Länge zum nächsten Punkt des Pfads
  l = path->getDistance(x0,y0);
  //Liegt der Roboter links oder rechts von der Verbindungsgeraden? Ggf. Negierung des Abstandsvektors
  if( path->Evaluate(x0,y0) > 5e-7 ) {
    l = -l;
  }
  
  //Evaluate gibt den Abstand zur Verbindungsgeraden des vorherigen und aktuellen Punktes an. Das wird ins Giofile geschrieben
  //giofile << l << " " << path->Evaluate(x0, y0) << " ";  // u 2 3

  //Differenz zwischen Orientierung des Pfades und der aktuellen Orientierung
  phic = phi0 - path->getAng();
  //Schreibe die aktuelle Orientierung, die Orientierung des Pfades und die Differenz ins gioFile
  //giofile << phi0 << " " << pathAng << " " << phic << " "; // using 4 5 6
  //Normalisiere Winkeldifferenz
  NormalizeAngle(phic);

  //Lokale Kopie der aktuellen Geschwindigkeit
  u = this->u0;

  w = Compute_W(l, phic, this->a, u, &err);
  double sign = w < 0 ? -1 : 1;
  w = (fabs(w) > this->Vm / this->AXIS_LENGTH) ? sign*(this->Vm / this->AXIS_LENGTH) : w;
  //w = (fabs(w) > this->Vm / this->AXIS_LENGTH) ? fabs(this->Vm / this->AXIS_LENGTH) : w;

  //giofile << w << " ";
  vright = u + AXIS_LENGTH * w * 0.5; //Geschwindigkeit rechtes Rad = aktuelle Geschwindigkeit - w/maximale Krümmung (k_rmax)
  vleft  = u - AXIS_LENGTH * w * 0.5; //Geschwindigkeit linken Rad = aktuelle Geschwindigkeit + w/maximale Krümmung (k_rmax)
  return 1;
}