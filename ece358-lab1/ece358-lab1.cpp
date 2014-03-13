//============================================================================
// Name        : ece358-lab1.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <list>
using namespace std;

double unviform_rv() {
	return ((double) rand() / ((double) RAND_MAX + 1));
}

double exponential_rv(double lambda) {
	//X = exp(-1*lambda*V);
	//V = -1*ln(X)/lambda
	return (double) -log(unviform_rv()) / (double) lambda;
}

namespace Sim {
enum EventType {
	ARRIVAL, DEPARTURE, OBSERVATION
};

struct simEvent {
	EventType type;
	int id;
	double packetLength;
	double time;
	bool dropped;
} SimEvent;

bool compare_times(simEvent& first, simEvent& second) {
	return first.time < second.time;
}
;

class Simulator {
	int Na; //Arrivals
	int Nd; //Departures
	int No; //Observations
	int linkRate;
	int queueSize;
	int durationT;
	int avgPacketSize;
	list<simEvent> ES;

public:
	/////////STATS/////
	int num_observations;
	int num_packets;
	double num_packets_in_buffer;
	double sojourn_time;
	double pIdle;
	double pLoss;
	//////////////////

	Simulator(int linkRate, int queueSize, int durationT, int L) {
		this->linkRate = linkRate;
		this->queueSize = queueSize;
		this->durationT = durationT;
		this->avgPacketSize = L;
		this->num_packets_in_buffer = 0;
		this->sojourn_time = 0;
		this->pIdle = 0;
		this->pLoss = 0;
	}
	~Simulator() {
		ES.clear();
	}
	void generate_observations(int alpha) {
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
	void generate_arrivals(int lambda) {
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

			sojourn_time += it->time;
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

int main(void) {
	srand(time(NULL));
	double p = 1.2;
	printf("p\tT\tlambda\talpha\tbuffer\tsojourn\tidle\tpLoss\t\n");
	//for (double p = 0.35; p < 0.85; p += 0.1) {
	//int k = 0;
	list<int> k_values;
	k_values.push_back(5);
	k_values.push_back(10);
	k_values.push_back(40);
	for (list<int>::iterator ki = k_values.begin(); ki != k_values.end(); ++ki) {
		int k = *ki;
		printf("k=%d\n", k);
		//for (double p = 0.4; p < 2; p += 0.1) {
		//for (double p = 2; p < 5; p += 0.2) {
		for (double p = 5; p < 10; p += 0.4) {
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
	}
//	int i = 0;
//	do {
//		i++;
//		printf("%f\n", poisson_rv(1/75.0));
//	} while (i < 1000);
	return EXIT_SUCCESS;
}
;
