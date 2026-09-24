#include <giovanni/gio_path.h>

using namespace std;

// Setzt die Standartwerte zum ausführen des Codes
void CGioController::InitDefault(){
  this->loop_exit    = 0;
  // Axenlänge
  this->AXIS_LENGTH  = 0.485;
  // Maximalgeschwindigkeit pro Rad (Betrag)
  this->Vm           = 1; 
  // Max estimation/measurement error für y
  this->d_y          = 0.001;
  // Max estimation/measurement error für theta
  this->d_th         = 0.34; 
  // Maximale Krümmung
  this->kr_max       = 2/AXIS_LENGTH;
  // Fixed linear speed u0
  this->u0           = Vm/2.0;
  // Skalierungsfaktor alpha für Beziehung zwischen gamma und h
  this->a            = 1.2;
  // Sehr kleine Zahl
  this->epsilon      = 1e-4;
  // Pfad dem gefolgt werden soll
  this->path         = new CCurve();
  // Initial pose x=y=0, theta=0
  this->setPose(0.0, 0.0, 0);
  // Origin für Startpunkt
  this->setLocalSystem(0.0);
}

// Rotationsmatrix?
void  CGioController::setLocalSystem(double ang){
  ex[0] = cos(ang); 
  ex[1] = sin(ang); 
  ey[0] = -sin(ang); 
  ey[1] = cos(ang); 
}

// Gain tuning der Werte h und gamma für die Bestimmung von omega
// Fall: y sehr klein (kleiner als epsilon) -> Equation 20
double  CGioController::H_case_1(double y, double theta, double u, double alpha, double *gamma){
  double h_j = SQR(kr_max) / (SQR(theta) * SQR(1+2*alpha));
  *gamma = 2*alpha*u*sqrt(h_j);
  return h_j;
};

// Fall: y größer epsilon -> Equation 21
double  CGioController::H_case_2(double y, double theta, double u, double alpha, double *gamma) {
  double h_j = 0.5 * ( - SQR(theta)/SQR(y) + sqrt( SQR(SQR(theta)/SQR(y)) + 4 * SQR(kr_max) / ( SQR(y) * SQR(1 + 2*alpha) ) ) );
  *gamma = 2*alpha*u*sqrt(h_j);
  return h_j;
};

double  CGioController::Compute_W(double y, double theta, double a, double u, int *err){
  double res, h, gamma;

  // Abstand zum vereinfachten Fall "Folge gerader Linie" ist nah genug an der 0
  if(fabs(y) < epsilon) {
    // Absolutwert des Winkels überschreitet den maximalen Fehler für theta
    if(fabs(theta) >= d_th) {
      //cerr << "H1_1\n";
      // Fall 1 mit normalem theta auswerten
	    h = H_case_1(y, theta, u, a, &gamma);
      *err = 0;
    // Winkel im Rahmen des maximalen Messfehlers
    } else {
      //cerr << "H1_2\n";
      // Fall 1 mit d_th auswerten, weil theta sonst zu klein
	    h = H_case_1(y, d_th, u, a, &gamma);
	    *err = 0;
    }
  // Sonst:
  } else {
    // Absolutwert des Winkels überschreitet den maximalen Fehler für y
    if(fabs(y) >= d_y) {
      // Fall 2 mit y und kleinem theta auswerten
	    h = H_case_2(y, d_th, u, a, &gamma);
	    *err = 0;
    } else {
      // Checken, ob theta kleiner als Fehler ist
	    if(fabs(theta) < d_th) {
        // Ja: Fall 2 mit kleinem y und kleinem theta auswerten
	      h = H_case_2(d_y, d_th, u, a, &gamma);
	      *err = 0;
      } else {
        // Nein: Fall 2 mit kleinem y und aktuellem theta auswerten
        h = H_case_2(d_y, theta, u, a, &gamma);
        *err = 0;
      }
    }
  }

  // Formeln für Controll-law von omega, abgeleitet von Gleichung 28
  // Fall: "Normal"
  if(fabs(theta) >= d_th) {
    res = -h * u * y * sin(theta)/theta - gamma * theta;
  // Fall: "Kleine Winkel"
  } else {
    // Fall: Winkel liegt zwischen aktuellem Fehler für Theta und Fehler zum Zeitpunkt t=0
    if(fabs(theta) <= d_th && fabs(theta) >= d_th_0){
	  res = -h * u * y - gamma * theta;
    // Fall: Sonst
    } else {
	  res = -h * u * y;
    }
  }

  // Rückgabe des Wertes für die Winkelgeschwindigkeit omega
  return res;
}

/*
  Constructor
  Setzten der Default-Werten,
  danach erstellen eines Files namens pos.dat.
  "pos.dat" enthält später alle Positionsdaten
*/
CGioController::CGioController(){
  this->InitDefault();
  giofile.open("pos.dat");
};

/*
  Destructor
  Schließt das zuvor geöffnete File und entfernt die path Variable
*/
CGioController::~CGioController(){
  giofile.flush();
  giofile.close();
  giofile.clear();  
  delete path;
}

/*
  Ändern der Axenlänge vom default-Wert
*/
void CGioController::setAxisLength(double val) {
  if(fabs(val) > 0) {
    //Ändere länge
    this->AXIS_LENGTH = fabs(val);
    //Ändere maximale Krümmung
    this->kr_max = 2/AXIS_LENGTH;
  }
}

// Getter für Axenlänge
double CGioController::getAxisLength() {
  return this->AXIS_LENGTH;
}

/*
  Ändert die linear velocity u0
  Absolutes ändern für abs = 1, sonst relatives ändern
*/
void CGioController::setCurrentVelocity(double val, int abs) {
  if(abs) {
    // Ersetzt u0 mit Wert val
    this->u0 = val;
  } else {
    this->u0 = (this->u0>val)?(val+this->u0):(this->u0+val);
  }
  /*
    Neue max Geschwindigkeit pro Rad wird festgelegt
    Mal 2, da sich u aus dem Mittelwert von v_l und v_r zusammesetzt
  */
  this->Vm = this->u0 * 2.0;
}

// Getter für current linear velocity u0
double CGioController::getCurrentVelocity() {
  return this->u0;
}

// Ändern der Pose mit anschließender Normalisierung des Winkels phi
void CGioController::setPose(double x, double y, double phi) {
  this->x0 = x;
  this->y0 = y;
  this->phi0 = phi;
  NormalizeAngle(this->phi0);
}

// Getter für Pose
void CGioController::getPose(double &x, double &y, double &ph) {
  x = this->x0;
  y = this->y0;
  ph = this->phi0;
}

// Generiert einen Pfad anhand der Daten im übergebenen File
int CGioController::getPathFromFile(const char* fname){
  int res = path->LoadFromFile(fname);
  if(res){
    path->initTraversal();
    this->loop_exit = path->getNext();
  }
  return res;
}

// Prüft, ob mindestens ein Punkt im Pfad weit geung weg vom Ende ist, damit die Route fahrbar ist.
int CGioController::canDetermineRobotPosition(int looped){
  int exit;
  int check_prev;

  exit       = 0;
  check_prev = 0;

  // Durchlaufen des files
  while(this->loop_exit != 0 && !exit) {
    if(path->pointInn(x0,y0)) {
      // Abstand zu (x0, y0) größer 0.1
	    if(path->getDistanceToEnd(x0, y0) > 0.1) {
        // Schleifenabbruch 
	      exit = 1;
	      giofile << path->getDistanceToEnd(x0, y0) << " "; // u 1
	      continue;
	    }
    }
    this->loop_exit = path->getNext(looped);
  }
  
  // exit = 1, wenn mindestens Punkt einen Größeren Abstand als 0.1 zum Endpunkt hat
  return exit;
}

/*
  Aufschreiben der wichtigen Informationen der States ins giofile
  Struktur:
  1: Distanz zum Ziel,
  2: Distanz zum nächsten Punkt?, 3: Evaluate(x0, y0)
  4: Winkel Roboter, 5: Winkel Pfad, 6: Differenz beider Winkel
  7: Winkelgeschwindigkeit omega
*/
int CGioController::getNextState(double &u, double &w, double &vleft, double &vright, int looped){
  double l, phic = 0, pathAng = 0, tmpw;
  int err;
  
  // Unternimmt nichts, wenn Roboter bereits nah genug am Ziel ist
  if(!canDetermineRobotPosition(looped)) {
    u = 0;
    w = 0;
    vleft = 0;
    vright = 0;
    return 0;
  }
	
  l = path->getDistance(x0,y0);
  if( path->Evaluate(x0,y0) > 5e-7 ) {
    l = -l;
  }
  
  // Eintragen der Distanz l ins giofile
  giofile << l << " " << path->Evaluate(x0, y0) << " ";  // u 2 3

  // Differenz Orientierung Eigen und Orientierung Pfad
  phic = phi0 - path->getAng();
  // Eintragen der Winkel und Differenz ins giofile
  giofile << phi0 << " " << pathAng << " " << phic << " "; // using 4 5 6
  NormalizeAngle(phic);

  // Setzen der linear velocity
  u = this->u0;

  // Omega berechnen
  w = Compute_W(l, phic, this->a, u, &err);
  // Vorzeichen der Winkelgeschwindigkeit bestimmen
  double sign = w < 0 ? -1 : 1;
  w = (fabs(w) > this->Vm / this->AXIS_LENGTH) ? sign*(this->Vm / this->AXIS_LENGTH) : w;
  //w = (fabs(w) > this->Vm / this->AXIS_LENGTH) ? fabs(this->Vm / this->AXIS_LENGTH) : w;

  giofile << w << " ";
  // Festlegen der Rädergeschwindigkeiten relativ zur linear velocity und angular velocity
  vright = u - AXIS_LENGTH * w * 0.5;
  vleft  = u + AXIS_LENGTH * w * 0.5;
  return 1;
}
