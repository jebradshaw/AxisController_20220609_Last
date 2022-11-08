// eprom memmory addres map for Scorbot 6-Axis controller

#ifndef EPROM_ADDRESS_H
#define EPROM_ADDRESS_H

volatile int BLOCK_SIZE = 32;

volatile int eprom_status = 0;
volatile int eprom_mot1maxI = 1 * BLOCK_SIZE;
volatile int eprom_mot2maxI = 2 * BLOCK_SIZE;
volatile int eprom_mot3maxI = 3 * BLOCK_SIZE;
volatile int eprom_mot4maxI = 4 * BLOCK_SIZE;
volatile int eprom_mot5maxI = 5 * BLOCK_SIZE;
volatile int eprom_mot6maxI = 6 * BLOCK_SIZE;

volatile int eprom_mode = 7 * BLOCK_SIZE;

#endif