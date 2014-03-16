//============================================================================
// Name        : ece358-lab2.cpp
// Author      : David Skinner
// Version     : 1.0
// Copyright   : All the things below, are belong to me
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <list>
#include <utility>
using namespace std;

//NEXT_EXPECTED_FRAME - represents the frame sequence number that it is expecting.
//==NEF -> forward to layer 3
//NEXT_EXPECTED_FRAME by 1 (modulo 2)
//Send Reverse ACK (send ack frame)
//RN equal to the updated value of NEXT_EXPECTED_FRAME

//If out of order:
//ACK with the current value of NEXT_EXPECTED_FRAME as RN. (not incremented)

//Sender:
//the sender maintains a counter NEXT_EXPECTED_ACK which is set to SN+1 (modulo 2)
//sequence-number of the latest frame sent by the sender
//RN = NEXT_EXPECTED_ACK (OKAY)
//informs the upper layer that it is ready to send a new packet.

//frame is correct but RN is not equal to NEXT_EXPECTED_ACK or if the frame contains errors,
//1) the sender does not do anything,
//2) resends the packet in the buffer in a new data frame with the same SN

//does not receive an ack before the frame’s timeout,
//either because the ACK was not sent by the receiver or was lost
//or because the timeout was too short, the sender will
//---->retransmit the same packet in a new frame carrying the same sequence number SN as the original

//GBN

//window size N
//number the frames sequentially modulo (N+1)
//buffer which is able to contain N packets and keeps the packets till they are acknowledged in the order they have been sent
//The sender has a pointer P that points to the frame sequence number of the oldest packet not yet acknowledged or to NIL

//sender, cumulative mark acknowledge up to RN
//GBN sender has a set of expected RNs for an upcoming ACK

//time-out of duration Δ
//lowest number packet sent to the channel, timout = tp+delta
//The sender responds to a time-out event by retransmitting all packets in the buffer
//New timeout event

//Update timeout
//1) Window slides (P changes)
//2) Retransmission

//Reciever:
//Similar to ABP

//acknowledge the correctly received frame by sending an ACK with RN = NEXT_EXPECTED_FRAME
//RCV 4, EXPECT=3
//ACK RN=3
//Discard 4

/*
 tc = 0 (current_time)
 SN = 0
 NEXT_EXPECTED_ACK = 1

 packet length L=H+l
 *fixed length*

 //transmit time:
 * tc + L/C

 for each packet, add a timeout event right away at
 tc + L/C + Δ

 call forward channel
 Event SEND()
 :returns ACK that must be added to ES
 or NIL (loss)
 :implements
 receiver
 //remove event from ES
 //update tc
 //if time-out
 //packet with SN sent to the channel in new frame
 //there can be only one time-out in the ES
 //else if ACK without ERROR
 //SN++
 //NEXT_EXPECT++

 //any outstanding time-outs in ES
 //have to be purged and a new time-out at tc + L/C + Δ has to be registered


 reverse channel


 2)
 Frame or Ack Lost:
 P = PLOSS
 Arrives: 1-P
 Error:
 ERROR/NOERROR

 Pnoerror = (1−BER)^L
 Ploss = 1 - Pnoerror - Perror

 Perror = sum_k=1:4{(L_choose_K)BER^k(1-BER)^(L-k)}

 We expect you to run L iterations.
 1-> probability BER
 Count errors for L bits

 LOSS - 5 or more bit errors to be a lost frame
 ERROR - 1 to 4 bits in error
 OK - 0

 propagation delay τ

 Input: tc, SN, L
 returns:
 NIL (lost)
 t+Tau, flag (error), SN(unchanged)
 */

double uniform_rv() {
	return ((double) rand() / ((double) RAND_MAX + 1));
}

namespace Sim {

/*
 Event types:

 TIME-OUT
 ACK

 Event:
 type (T/A)
 time
 sequence-number
 error-flag (error)
 */
enum EventType {
	TIMEOUT, ACK
};

enum ErrorType {
	NONE, ERROR, LOSS
};

struct simEvent {
	EventType type;
	double time;
	double SN;
	bool error;
} SimEvent;

bool compare_times(simEvent& first, simEvent& second) {
	return first.time < second.time;
}
;
bool timeout_event(simEvent& x) {
	return (bool)(x.type == TIMEOUT);
}
;

class ABPSimulator {
	int H, l, C;
	double Delta, Tau, BER; //current-time
	list<simEvent> ES;

public:
	/////////STATS/////
	double tc;
	int num_packets;
	int num_sent_success;
	int NEXT_EXPECTED_FRAME;
	int NEXT_EXPECTED_ACK;
	int SN;
	//////////////////

	ABPSimulator(int H, int l, double Delta, int C, double Tau, double BER,
			int num_packets) {
		this->H = H;
		this->l = l;
		this->Delta = Delta;
		this->C = C;
		this->Tau = Tau;
		this->BER = BER;
		this->num_packets = num_packets;
		this->tc = 0;
		/*
		 * INITIAL VALUES
		 */
		this->num_sent_success = 0;
		this->SN = 0;
		this->NEXT_EXPECTED_ACK = 1;
	}
	~ABPSimulator() {
		ES.clear();
	}

	void es_sorted_insert(simEvent event) {
		list<simEvent>::iterator next = ES.begin();
		while (next != ES.end()) {
			if (next->time > event.time) {
				ES.insert(next, event);
				return;
			}
			++next;
		}
		ES.push_back(event);
	}

	void send_packets() {
		addTimeout();
		while (this->num_sent_success < this->num_packets) {
			this->tc += ((double) this->l / (double) this->C) + this->Tau;
			SEND(this->tc, this->SN);
			simEvent latest = getEvent();
			if (latest.type == TIMEOUT) {
				//printf("TIMEOUT SN %d tc %f \r\n", this->SN, this->tc);
				//fflush(stdout);
				this->tc = latest.time;
				addTimeout(); //Add a new timeout
			} else if (latest.SN == this->NEXT_EXPECTED_ACK) {
				//printf("OK SN %d tc %f \r\n", SN, this->tc);
				//fflush(stdout);
				this->NEXT_EXPECTED_ACK = (this->NEXT_EXPECTED_ACK + 1) % 2;
			} else {
				//retransmit
			}
		}
	}

	void purgeTimeouts() {
		ES.remove_if(timeout_event);
	}

	void addTimeout() {
		purgeTimeouts();
		simEvent* timeout = new simEvent();
		timeout->time = this->tc
				+ (double) (this->l + this->H) / (double) this->C
				+ (double) this->Delta;
		this->es_sorted_insert(*timeout);
	}

	simEvent getEvent() {
		list<simEvent>::iterator next = ES.begin();
		while (next != ES.end()) {
			if (next->time >= this->tc) {
				return *next;
			}
			++next;
		}
		return *next;
	}

	void SEND(double time, int SN) {
		ErrorType ack_value = this->RECIEVE(time, this->SN);

		if (ack_value != LOSS) {
			this->tc += this->Tau + ((double)this->H / (double) this->C);
			//NOT LOST - ADD ACK TO ES
			//NO ERROR: INCREMENT SN
			if (ack_value == NONE) {
				SN = (SN + 1) % 2;
				this->SN = SN;
			}
			//ELSE: ERROR ACK PREVIOUS
			simEvent* ack = new simEvent();
			ack->type = ACK;
			ack->SN = SN;
			ack->time = this->tc;
			ack->error = (bool) (ack_value == ERROR);
			this->es_sorted_insert(*ack);
		}else{
			printf("LOSS SN %d tc %f \r\n", SN, this->tc);
			fflush(stdout);
		}
		return;
	}

	int bits_in_error() {
		/*
		 We expect you to run L iterations.
		 1-> probability BER
		 Count errors for L bits
		 */
		int count = 0;
		for (int i = 0; i < this->l; i++) {
			count += (uniform_rv() > this->BER) ? 0 : 1;
		}
		return count;
	}

	ErrorType RECIEVE(double tc, int SN) {
		//Calculate bits in error, and flag packet
		int BIE = bits_in_error();
		// LOSS - 5 or more bit errors to be a lost frame
		//ERROR - 1 to 4 bits in error
		//OK - 0
		if (BIE >= 5) {
			return LOSS;
		} else if (BIE > 1) {
			//ERROR
			return ERROR;
		} else {
			if(SN == NEXT_EXPECTED_FRAME){
				NEXT_EXPECTED_FRAME = (NEXT_EXPECTED_FRAME + 1) % 2;
				this->num_sent_success++;
			}
			//OK
		}
		return NONE;
	}
};

}
void question_1(double BER){
	printf("BER: %f\n", BER);
	printf("H\tL\tDelta\tC\tTau\tBER\t#Packets\tThroughput\n");
	int C = 5 * pow(1024, 2);
	int num_packets = 10000;
	int L = 1500;
	int H = 54;
	double Tau = 0.010;
	for (double x = 2.5; x < 13; x+=2.5){
		double Delta = x * Tau;
	//double Tau = 0.500;
		Sim::ABPSimulator* sim = new Sim::ABPSimulator(H, L, Delta, C, Tau, BER,
				num_packets);
		sim->send_packets();
		printf("%d\t%d\t%f\t%d\t%f\t%f\t%d\t",H,L, Delta, C, Tau, BER, num_packets);
		printf("%f\n", ((double)num_packets * (double)L) / (double) sim->tc);
		fflush(stdout);
		delete sim;
	}
}

void question_2(double BER){
	printf("BER: %f\n", BER);
	printf("H\tL\tDelta\tC\tTau\tBER\t#Packets\tThroughput\n");
	int C = 5 * pow(1024, 2);
	int num_packets = 10000;
	int L = 1500;
	int H = 54;
	double Tau = 0.010;
	for (double x = 2.5; x < 13; x+=2.5){
		double Delta = x * Tau;
	//double Tau = 0.500;
		Sim::ABP_NAK* sim = new Sim::ABP_NAK(H, L, Delta, C, Tau, BER,
				num_packets);
		sim->send_packets();
		printf("%d\t%d\t%f\t%d\t%f\t%f\t%d\t",H,L, Delta, C, Tau, BER, num_packets);
		printf("%f\n", ((double)num_packets * (double)L) / (double) sim->tc);
		fflush(stdout);
		delete sim;
	}
}


int main(void) {
	srand(time(NULL));
	question_1(0.0);
	question_1(pow(10,-5));
	question_1(pow(10,-4));
	return 0;
//k_values	.push_back(5);
//	k_values.push_back(10);
//	k_values.push_back(40);
//	question_1();
//	question_3();
//	question_4();
//	question_6();
}
;

