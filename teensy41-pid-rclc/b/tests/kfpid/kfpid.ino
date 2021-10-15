#include "Filter.hpp"
#include "kf_pid.hpp"
#include <Encoder.h>
// #include <math.h>

#define DT 10
#define IPR 1836

Encoder enc(2, 3);

Filter_t kalman;

Regulator pid;

float raw = 0; // rounds
float praw = 0;
float tgt_V = 0;

void setup() {
  delay(1000);
  Serial6.begin(115200);
}

void loop() {

  raw = enc.read() / (float)IPR;
  kalman.update(raw);
  // tgt_V = floor(millis() / 3000) * (2);   //DBG
  pid.updateTgt(tgt_V);
  pid.updateRes(kalman.getX(0), kalman.getX(1), kalman.getX(2));

  Serial6.print(kalman.getX(1));
  Serial6.print(" ");
  Serial6.println(pid.getRes());

  praw = raw;
  delay(DT);
}

//
