#include "Arduino.h"
#include "simulation_plant_v6.h"
#include <assert.h>
uint32_t testClock=0;
HardwareSerial Serial;
int main(){
 SimulationPlantV6 plant;plant.begin(1000);uint16_t value=0;
 assert(plant.stopped()&&plant.raw(0)==1000&&plant.read(2,0xF603,value)&&value==0);
 assert(plant.forwardSign(0)==-1&&plant.forwardSign(3)==-1);
 plant.command(0,10);for(uint32_t t=1020;t<=2200;t+=20)plant.tick(t);
 assert(plant.raw(0)<1000&&plant.frequency(0)==500);
 assert(!plant.write(0,0xF603,100));plant.stop();for(uint32_t t=2220;t<=2600;t+=20)plant.tick(t);assert(plant.stopped());
 assert(!plant.write(0,0xF402,0)&&!plant.write(0,0x2000,1));
 assert(plant.write(2,0xF603,1000));plant.command(2,10);plant.tick(2620);assert(plant.frequency(2)==0);
 plant.stop(true);assert(plant.write(2,0xF603,0));plant.command(2,20);plant.tick(3020);assert(plant.frequency(2)>0);
 plant.emergency(true);assert(plant.estopLatched&&plant.stopped()&&!plant.clearLatch());plant.emergency(false);assert(plant.estopLatched&&plant.clearLatch());
 plant.offlineMask=4;assert(!plant.read(2,0xF603,value));plant.offlineMask=0;
 plant.rate=5;const uint32_t previous=plant.clock();plant.tick(3040);assert(plant.clock()-previous==100);
 plant.command(1,-100);plant.tick(3060);assert(plant.raw(1)>=1000&&plant.frequency(1)>0);
 plant.stop(true);assert(plant.setPosition(0,9999)&&!plant.setPosition(4,0));plant.command(0,-100);
 for(uint32_t t=3080;t<50000;t+=20)plant.tick(t);assert(plant.raw(0)==10000&&plant.stopped());
 assert(Serial.requests==0);puts("PASS: virtual ramps, raw directions/P0.09, startup block, named writes, offline, E-STOP latch, rate, coordinate bounds; no I/O");
}
