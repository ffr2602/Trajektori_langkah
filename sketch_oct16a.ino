#include <DynamixelSerial.h>

#define tinggi 45.00
#define awalx 45.00
#define awaly 00.00
#define awalz 73.30

//================belakang, tengah, depan
int cox_kiri[] = { 14, 11, 2 };
int fem_kiri[] = { 13, 10, 1 };
int tib_kiri[] = { 12,  9, 0 };
//================belakang, tengah, depan
int cox_kanan[] = { 17, 8, 5 };
int fem_kanan[] = { 16, 7, 4 };
int tib_kanan[] = { 15, 6, 3 };

const float cx = 00.00;
const float fm = 45.00;
const float tb = 73.30;

float sudut_A, sudut_B, sudut_C;
float sudut_B1, sudut_B2;

float alpa, beta, gama;

int xp1, xp2, xp3, xp4;
int yp1, yp2, yp3, yp4;
int zp1, zp2, zp3, zp4;

float iterasi = 0.05;

void setup() {

  Dynamixel.setSerial(&Serial3);  // Rx3 Tx3
  Dynamixel.begin(115200, 2);
  //Serial.begin(9600);
}

void posisi(float x, float y, float z) {

  float rad = 180 / PI;

  float p = sqrt((x * x) + (y * y));
  float p01 = (p - cx);
  float p02 = sqrt((p01 * p01) + (z * z));

  sudut_B1 = acos(((fm * fm) + (p02 * p02) - (tb * tb)) / (2 * fm * p02)) * rad;
  sudut_B2 = atan(p01 / z) * rad;
  sudut_A = atan(y / x) * rad;
  sudut_B = sudut_B1 + sudut_B2;
  sudut_C = acos(((fm * fm) + (tb * tb) - (p02 * p02)) / (2 * fm * p02)) * rad;

  alpa = sudut_A;
  beta = sudut_B - 90;
  gama = 90 - sudut_C;

  Serial.print("alpa : ");
  Serial.print(alpa);
  Serial.print(",");
  Serial.print("beta : ");
  Serial.print(beta);
  Serial.print(",");
  Serial.print("gama : ");
  Serial.println(gama);
  delay(1);
  kaki();
}

uint16_t convert(int sudut) {

  return map(sudut, -180, 180, 0, 1023);
}

void kaki() {

  Dynamixel.move(cox_kanan[0], convert(alpa));
  Dynamixel.move(fem_kanan[0], convert(beta));
  Dynamixel.move(tib_kanan[0], convert(gama));

  // Dynamixel.move(cox_kanan[1], convert(alpa));
  // Dynamixel.move(fem_kanan[1], convert(beta));
  // Dynamixel.move(tib_kanan[1], convert(gama));

  Dynamixel.move(cox_kanan[2], convert(alpa));
  Dynamixel.move(fem_kanan[2], convert(beta));
  Dynamixel.move(tib_kanan[2], convert(gama));

  Dynamixel.move(cox_kiri[0], convert(alpa));
  Dynamixel.move(fem_kiri[0], convert(beta));
  Dynamixel.move(tib_kiri[0], convert(gama));

  // Dynamixel.move(cox_kiri[1], convert(alpa));
  // Dynamixel.move(fem_kiri[1], convert(beta));
  // Dynamixel.move(tib_kiri[1], convert(gama));

  Dynamixel.move(cox_kiri[2], convert(alpa));
  Dynamixel.move(fem_kiri[2], convert(beta));
  Dynamixel.move(tib_kiri[2], convert(gama));
}


void tr_kaki_seret(float x, float y, float z) {

  xp1 = awalx;
  xp2 = xp1;
  xp3 = x;
  xp4 = xp3;

  yp1 = awaly;
  yp2 = yp1;
  yp3 = y;
  yp4 = yp3;

  zp1 = awalz;
  zp2 = zp1;
  zp3 = z;
  zp4 = zp3;

  for (float t = 0.00; t <= 1.00; t = t + iterasi) {

    float a = (1 - t) * (1 - t) * (1 - t);
    float b = 3 * t * (1 - t) * (1 - t);
    float c = 3 * t * t * (1 - t);
    float d = t * t * t;

    float px = (a * xp1) + (b * xp2) + (c * xp3) + (d * xp4);
    float py = (a * yp1) + (b * yp2) + (c * yp3) + (d * yp4);
    float pz = (a * zp1) + (b * zp2) + (c * zp3) + (d * zp4);

    // Serial.print("x :");
    // Serial.print(px);
    // Serial.print(",");
    // Serial.print("y :");
    // Serial.print(py);
    // Serial.print(",");
    // Serial.print("z :");
    // Serial.println(pz);

    posisi(px, py, pz);
  }
}

void tr_kaki_naik(float x, float y, float z) {

  xp1 = awalx;
  xp2 = xp1;
  xp3 = x;
  xp4 = xp3;

  yp1 = awaly;
  yp2 = yp1;
  yp3 = y;
  yp4 = yp3;

  zp1 = z;
  zp2 = tinggi;
  zp3 = zp2;
  zp4 = zp1;

  for (float t = 0.00; t <= 1.00; t = t + iterasi) {

    float a = (1 - t) * (1 - t) * (1 - t);
    float b = 3 * t * (1 - t) * (1 - t);
    float c = 3 * t * t * (1 - t);
    float d = t * t * t;

    float px = (a * xp1) + (b * xp2) + (c * xp3) + (d * xp4);
    float py = (a * yp1) + (b * yp2) + (c * yp3) + (d * yp4);
    float pz = (a * zp1) + (b * zp2) + (c * zp3) + (d * zp4);

    // Serial.print("x :");
    // Serial.print(px);
    // Serial.print(",");
    // Serial.print("y :");
    // Serial.print(py);
    // Serial.print(",");
    // Serial.print("z :");
    // Serial.println(pz);

    posisi(px, py, pz);
  }
}

void loop() {

 // posisi(45, 0, 50);
  tr_kaki_seret(45.00, 0.00, 73.30);

  tr_kaki_naik(45.00, 45.00, 73.30);

}