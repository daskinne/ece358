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
#include <pair>
#include <make_pair>
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
*/

double unviform_rv() {
	return ((double) rand() / ((double) RAND_MAX + 1));
}

double exponential_rv(double lambda) {
	//X = exp(-1*lambda*V);
	//V = -1*ln(X)/lambda
	return (double) -log(unviform_rv()) / (double) lambda;
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

class Simulator {
	int H, l, Delta, C;
	double tc, Tau, BER; //current-time
	list<simEvent> ES;

public:
	/////////STATS/////
	int num_packets;
	int num_sent_success;
	int NEXT_EXPECTED_FRAME;
	int NEXT_EXPECTED_ACK;
	int SN;
	//////////////////

	Simulator(int H, int l, int Delta, int C, double Tau, double BER,
			int num_packets) {
		this->H = H;
		this->l = l;
		this->Delta = Delta;
		this->C = C;
		this->Tau = Tau;
		this->BER = BER;
		this->num_packets = num_packets;
		this->tc = 0;
		this->num_sent_success = 0;
	}
	~Simulator() {
		ES.clear();
	}

	void send_packets(){
		while(this->num_sent_success < this->num_packets){
			simEvent packet;
			packet.SN = this->SN+1 %2;
			packet.time=this->tc + (double)this->l/(double)this->C;
			simEvent timeout;
			packet.time = this->tc +(double)(this->l+this->H)/(double)this->C + (double)this->Delta;
			ES.push_back(timeout);
			simEvent ack = this->SEND(packet);
			if (ack != 0){
				ES.push_back(ack);
			}
		}
	}

	simEvent SEND(simEvent packet){
		 //returns ACK that must be added to ES
		 //or NIL (loss)
		 //:implements
		 //receiver
		this->tc = packet.time;

		//remove event from ES
		 //update tc
		 //if time-out
		 //packet with SN sent to the channel in new frame
		 //there can be only one time-out in the ES
		 pair<double,bool> ACK = this->RECIEVE(this->tc, this->SN);
		 if (ACK.second){
			 //ERROR
		 }else{
			 this->SN = this->SN+1%2;
			 this->NEXT_EXPECTED_FRAME = this->NEXT_EXPECTED_FRAME +1%2;
			 this->num_sent_success++;
			 simEvent ack;
			 ack.SN=this->SN;
			 ack.time=pair.first;
			 return ack;
		 }
		 return 0;
		 //else if ACK without ERROR
		 //SN++
		 //NEXT_EXPECT++

		 //any outstanding time-outs in ES
		 //have to be purged and a new time-out at tc + L/C + Δ has to be registered
	}

	int bits_in_error(){
		/*
		 Pnoerror = (1−BER)^L
		 Ploss = 1 - Pnoerror - Perror

		 Perror = sum_k=1:4{(L_choose_K)BER^k(1-BER)^(L-k)}

		 We expect you to run L iterations.
		 1-> probability BER
		 Count errors for L bits
		 */
		int count=0;
		for(int i = 0; i < this->l; i++){
			//TODO: change to random variable
			count += this->BER;
		}
		return count;
	}

	pair<double, bool> RECIEVE(double tc, int SN){
		//Calculate bits in error, and flag packet
		int BIE = bits_in_error();
		// LOSS - 5 or more bit errors to be a lost frame
		//ERROR - 1 to 4 bits in error
		//OK - 0
		bool error = false;
		if(BIE >= 5){
			return 0;
		}else if(BIE>1){
			//ERROR
			error=true;
		}else{
			//OK
		}
		return std::make_pair(tc+this->Tau,error);
	}

	void generate_observations(double alpha) {
		double time = 0;
		int i = 1;
		while (true) {
			simEvent e;
			e.type = OBSERVATION;
			e.id = i;
			e.dropped = false;
			time += exponential_rv(alpha);
			e.time = time;
			if (e.time > durationT) {
				break;
			}
			i++;
			ES.push_back(e);
		}
		num_observations = i;
	}
	void generate_arrivals(double lambda) {
		double time = 0;
		int i = 1;
		while (true) {
			simEvent e;
			e.type = ARRIVAL;
			e.id = i;
			e.dropped = false;
			e.packetLength = exponential_rv(1.0 / (double) avgPacketSize);
			time += exponential_rv(lambda);
			e.time = time;
			if (e.time > durationT) {
				break;
			}
			i++;
			ES.push_back(e);
		}
		num_packets = i;
	}

	void calculate_departures() {
		double current_time = 0;
		int packet_count = 0;
		for (list<simEvent>::iterator it = ES.begin(); it != ES.end(); ++it) {
			if (it->type != ARRIVAL || it->dropped) {
				continue;
			}
			//How long has this packet been waiting?
			//printf("Packet at time %f (delta)%f: id %d size: %f \n", it->time, (current_time - it->time), it->id, it->packetLength);
			if (it->time > current_time) {
				current_time = it->time;
			}
			//add departure event at
			double departureTime = (double) current_time
					+ (it->packetLength / (double) linkRate);

			if (queueSize > 0) {
				//if finite queue, mark dropped packets by considering the number left in the queue while servicing
				//count # events between current time and departureTime, if > queueSize, drop packet
				int numQueued = 0;
				for (list<simEvent>::iterator dq = it; dq != ES.end(); ++dq) {
					if (dq->type != ARRIVAL || dq->dropped) {
						//only check arrivals and packets that have not yet been dropped
						continue;
					}
					if (dq->time > departureTime) {
						break;
						//We have moved beyond the affected interval
					}
					if (numQueued == queueSize) {
						//Start dropping packets
						//printf("Packet id %d dropped (time:%f)\n", dq->id, dq->time);
						dq->dropped = true;
						pLoss++;
					} else {
						numQueued++;
					}
				}
			}
			//add departure event
			simEvent e;
			e.id = it->id;
			e.type = DEPARTURE;
			e.time = departureTime;
			//printf("Packet departs at time %f id %d \n", e.time, e.id);
			ES.push_back(e);

			sojourn_time += departureTime - it->time;
			packet_count++;
			//time advances to take new packet
			current_time = departureTime;
		}
		//average sojourn time
		sojourn_time /= (double) packet_count;
		pLoss /= (double) num_packets;
	}

	void order_events() {
		ES.sort(compare_times);
	}

	void observe_event(simEvent* se) {
		switch (se->type) {
		case ARRIVAL:
			if (!se->dropped) {
				Na++;
			}
			break;
		case DEPARTURE:
			Nd++;
			break;
		case OBSERVATION:
			No++;
			num_packets_in_buffer += Na - Nd;
			if (Na == Nd) {
				pIdle++;
			}
			//printf("Time: %f No %d Na %d Nd %d\n", se->time, No, Na, Nd);
			break;
		}
	}

	void observe_events() {
		Na = 0;
		Nd = 0;
		No = 0;
		for (list<simEvent>::iterator it = ES.begin(); it != ES.end(); ++it) {
			observe_event(&*it);
		}
		pIdle /= num_observations;
		num_packets_in_buffer /= (double) num_observations;
	}

};
}
void run_simulation(int k, double p) {
	int T = 11000;
	int L = 12000;
	double C = pow(1024, 2);
	double lambda = p * C / (double) L;
	double alpha = lambda;
	int queue_size = k;
	Sim::Simulator* sim = new Sim::Simulator(C, queue_size, T, L);
	//Generate observation events, poisson parameter alpha ( if less than T)
	sim->generate_observations(alpha);
	//Packet arrival times (parameter lambda)
	sim->generate_arrivals(lambda);
	sim->order_events();
	//Find departure times
	sim->calculate_departures();
	//packet arrivals so far, packets departures so far
	sim->order_events();
	sim->observe_events();
	printf("%f\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t\n", p, T, lambda, alpha,
			sim->num_packets_in_buffer, sim->sojourn_time, sim->pIdle,
			sim->pLoss);
	//number of observations
	delete sim;
}
list<int> k_values;

void question_1() {
	printf("Question 1\r\n");
	int i = 0;
	do {
		i++;
		printf("%f\n", exponential_rv(75.0));
	} while (i < 1000);
}

void question_3() {
	printf("Question 3\r\n");
	printf("p\tT\tlambda\talpha\tbuffer\tsojourn\tidle\tpLoss\t\n");
	int k = 0;
	for (double p = 0.35; p < 0.85; p += 0.1) {
		run_simulation(k, p);
	}
}
void question_4() {
	printf("Question 4\r\n");
	printf("p\tT\tlambda\talpha\tbuffer\tsojourn\tidle\tpLoss\t\n");
	double p = 1.2;
	run_simulation(0, p);
}

void question_6() {
	printf("Question 4\r\n");
	printf("p\tT\tlambda\talpha\tbuffer\tsojourn\tidle\tpLoss\t\n");
	for (list<int>::iterator ki = k_values.begin(); ki != k_values.end();
			++ki) {
		int k = *ki;
		printf("k=%d\n", k);
		for (double p = 0.4; p < 2; p += 0.1) {
			run_simulation(k, p);
		}
		for (double p = 2; p < 5; p += 0.2) {
			run_simulation(k, p);
		}
		for (double p = 5; p < 10; p += 0.4) {
			run_simulation(k, p);
		}
	}
}

int main(void) {
	srand (time(NULL));

k_values	.push_back(5);
	k_values.push_back(10);
	k_values.push_back(40);
	question_1();
	question_3();
	question_4();
	question_6();
}
;
