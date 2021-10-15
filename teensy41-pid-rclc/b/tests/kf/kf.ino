#include "Filter.hpp"
#include <Encoder.h>
#define DT 10
#define IPR 1836
Encoder enc(2, 3);
Filter_t kalman;

void setup() {
  delay(1000);
  Serial6.begin(115200);
}
float raw = 0; // rounds
float praw = 0;
void loop() {
  raw = enc.read() / (float)IPR;
  kalman.update(raw);
  Serial6.print(kalman.getX(1));
  Serial6.print(" ");
  Serial6.print((raw - praw) * 100);
  Serial6.print(" ");
  Serial6.print(kalman.getX(0));
  Serial6.print(" ");
  Serial6.println(raw);
  praw = raw;
  delay(DT);
}
