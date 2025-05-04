#include <stdlib.h>
#include <stdio.h>
#include "emulator.h"
#include "sr.h"

#define TRUE 1
#define FALSE 0

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 12      /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for (i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);

  return checksum;
}

int IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return FALSE;
  else
    return TRUE;
}

/********* Sender (A) variables and functions ************/

static struct pkt buffer[SEQSPACE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static int acked[SEQSPACE];            /* array to track if a packet was acknowledged */
static float time_sent[SEQSPACE];      /* array to store the time each packet was sent */
static int timer_active[SEQSPACE];     /* array to track which timers are active */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i, j;
  int any_timer_running;

  if (((A_nextseqnum - windowfirst + SEQSPACE) % SEQSPACE) < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* buffer and track the packet */
    buffer[A_nextseqnum] = sendpkt;
    acked[A_nextseqnum] = FALSE;
    timer_active[A_nextseqnum] = TRUE;

    /* send packet to network layer */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer if no timer is running */
    any_timer_running = FALSE;
    for (j = 0; j < SEQSPACE; j++) {
      if (timer_active[j]) {
        any_timer_running = TRUE;
        break;
      }
    }
    if (!any_timer_running) {
      starttimer(A, RTT);
    }

    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
    windowcount++;
  }
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}

/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int i;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    if (!acked[packet.acknum]) {
      acked[packet.acknum] = TRUE;
      new_ACKs++;
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n", packet.acknum);
    } else {
      if (TRACE > 0)
        printf("----A: duplicate ACK received, do nothing!\n");
    }

    /* slide window forward as far as possible */
    while (windowcount > 0 && acked[buffer[windowfirst].seqnum]) {
      acked[buffer[windowfirst].seqnum] = FALSE;
      timer_active[buffer[windowfirst].seqnum] = FALSE;
      windowfirst = (windowfirst + 1) % SEQSPACE;
      windowcount--;
    }

    stoptimer(A);
    if (windowcount > 0)
      starttimer(A, RTT);
  } else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}


/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i, seq;
  int first = 1;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  for (i = 0; i < windowcount; i++) {
    seq = (windowfirst + i) % SEQSPACE;
    if (timer_active[seq] && !acked[seq]) {
      if (TRACE > 0)
        printf("---A: resending packet %d\n", seq);
      tolayer3(A, buffer[seq]);
      packets_resent++;
      if (first) {
        starttimer(A, RTT);
        first = 0;
      }
    }
  }
}


/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  A_nextseqnum = 0;
  windowfirst = 0;
  windowlast = -1;
  windowcount = 0;

  for (i = 0; i < SEQSPACE; i++) {
    acked[i] = FALSE;
    time_sent[i] = 0.0;
    timer_active[i] = FALSE;
  }
}

/********* Receiver (B)  variables and procedures ************/

static struct pkt bufferB[SEQSPACE];
static int received[SEQSPACE];

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt ackpkt;
  int i, seq;

  /* if not corrupted and received packet is in order */
  if (!IsCorrupted(packet)) {
    if (!received[packet.seqnum]) {
      received[packet.seqnum] = TRUE;
      bufferB[packet.seqnum] = packet;

      if (TRACE > 0)
        printf("----B: packet %d is correctly received\n", packet.seqnum);
    }

    /* deliver in-order packets starting from expectedseqnum */
    while (received[expectedseqnum]) {
      if (TRACE > 0)
        printf("----B: packet %d delivered to application layer\n", expectedseqnum);

      tolayer5(B, bufferB[expectedseqnum].payload);
      received[expectedseqnum] = FALSE;
      expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
      packets_received++;
    }

    ackpkt.acknum = (expectedseqnum + SEQSPACE - 1) % SEQSPACE;
  } else {
   
    if (TRACE > 0)
      printf("----B: packet corrupted, resend ACK!\n");
    ackpkt.acknum = (expectedseqnum + SEQSPACE - 1) % SEQSPACE;
  }

  /* create packet */
  ackpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;

  
  for (i = 0; i < 20; i++)
    ackpkt.payload[i] = '0';

  /* compute checksum */
  ackpkt.checksum = ComputeChecksum(ackpkt);

  /* send out packet */
  tolayer3(B, ackpkt);
}



/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  expectedseqnum = 0;
  B_nextseqnum = 1;

  for (i = 0; i < SEQSPACE; i++) {
    received[i] = FALSE;
    bufferB[i].seqnum = NOTINUSE;
  }
}


/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
    
}
