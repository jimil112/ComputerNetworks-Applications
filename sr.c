#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"   /* Selective Repeat header */

/* ******************************************************************
   Selective Repeat protocol - Step 2: Sender buffer refactor.
   Based on Go-Back-N template from J.F. Kurose's network emulator.

   Modifications:
   - Replaced GBN sender window logic with SR per-packet tracking
   - Added send_buffer, packet_sent, packet_acked arrays
   - Updated A_output and A_init to support Selective Repeat
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of unacknowledged packets in SR */
#define SEQSPACE 7      /* the minimum sequence space must be at least WINDOWSIZE + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver */
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  for (int i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);
  checksum += packet.seqnum + packet.acknum;
  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  return packet.checksum != ComputeChecksum(packet);
}


/********* Sender (A) variables and functions ************/

/* SR sender tracking variables */
static struct pkt send_buffer[SEQSPACE];      /* circular buffer of sent packets */
static bool packet_sent[SEQSPACE];            /* whether packet i has been sent */
static bool packet_acked[SEQSPACE];           /* whether packet i has been acknowledged */
static int base;                              /* oldest unacknowledged seqnum */
static int nextseqnum;                        /* next sequence number to send */

/* A_init: called once before any other sender routine */
void A_init(void)
{
  base = 0;
  nextseqnum = 0;
  for (int i = 0; i < SEQSPACE; i++) {
    packet_sent[i] = false;
    packet_acked[i] = false;
  }
}

/* A_output: called by application layer to send a message */
void A_output(struct msg message)
{
  int window_size = (nextseqnum - base + SEQSPACE) % SEQSPACE;

  /* if within window, send packet */
  if (window_size < WINDOWSIZE) {
    struct pkt packet;
    packet.seqnum = nextseqnum;
    packet.acknum = NOTINUSE;

    for (int i = 0; i < 20; i++)
      packet.payload[i] = message.data[i];
    packet.checksum = ComputeChecksum(packet);

    send_buffer[nextseqnum] = packet;
    packet_sent[nextseqnum] = true;
    packet_acked[nextseqnum] = false;

    tolayer3(A, packet);
    if (TRACE > 0)
      printf("A_output: Sent packet %d\n", nextseqnum);

    /* start timer if base == nextseqnum (first in window) */
    if (base == nextseqnum)
      starttimer(A, RTT);

    nextseqnum = (nextseqnum + 1) % SEQSPACE;
  } else {
    if (TRACE > 0)
      printf("A_output: Window full. Packet dropped.\n");
    window_full++;
  }
}


/* A_input: called when ACK packet is received from B */
void A_input(struct pkt packet)
{
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;
    new_ACKs++;
    /* Per-packet ACK processing will be added in Step 3 */
  } else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, ignoring.\n");
  }
}


/* A_timerinterrupt: called when timer expires */
void A_timerinterrupt(void)
{
  if (TRACE > 0)
    printf("----A: timer interrupt, to be handled in Step 3\n");
  /* In Step 3 we’ll selectively retransmit only timed-out packets */
}


/********* Receiver (B) - still GBN-style for now ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */

/* B_input: called when packet arrives at receiver from layer 3 */
void B_input(struct pkt packet)
{
  struct pkt sendpkt;

  /* if not corrupted and in-order */
  if (!IsCorrupted(packet) && packet.seqnum == expectedseqnum) {
    if (TRACE > 0)
      printf("----B: packet %d correctly received, sending ACK\n", packet.seqnum);
    packets_received++;

    tolayer5(B, packet.payload);

    sendpkt.acknum = expectedseqnum;
    expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
  } else {
    /* if corrupted or unexpected, resend last ACK */
    if (TRACE > 0)
      printf("----B: packet corrupted or unexpected, sending duplicate ACK\n");
    sendpkt.acknum = (expectedseqnum == 0) ? SEQSPACE - 1 : expectedseqnum - 1;
  }

  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;

  for (int i = 0; i < 20; i++)
    sendpkt.payload[i] = '0';

  sendpkt.checksum = ComputeChecksum(sendpkt);

  tolayer3(B, sendpkt);
}


/* B_init: called once before any other receiver routine */
void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;
}

/* B_output is unused in this practical */
void B_output(struct msg message) {}
void B_timerinterrupt(void) {}
