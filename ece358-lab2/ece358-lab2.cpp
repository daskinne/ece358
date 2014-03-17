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
#include <set>
#include <utility>
using namespace std;

double uniform_rv() {
	return ((double) rand() / ((double) RAND_MAX));
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

struct Packet {
	int SN;
	double timeout;
} SimPacket;

struct simEvent {
	EventType type;
	double time;
	int SN;
	bool error;
} SimEvent;

bool compare_times(simEvent& first, simEvent& second) {
	return first.time < second.time;
}
;
bool timeout_event(simEvent& x) {
	return (bool) (x.type == TIMEOUT);
}
;

class ABPSimulator {
	int H, l;
	double C, Delta, Tau, BER; //current-time
	list<simEvent> ES;

public:
	/////////STATS/////
	double tc;
	double total_time;
	int num_packets;
	int num_sent_success;
	int NEXT_EXPECTED_FRAME;
	int NEXT_EXPECTED_ACK;
	bool NACK;
	//////////////////

	ABPSimulator(int H, int l, double Delta, double C, double Tau, double BER,
			int num_packets, bool NACK) {
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
		this->NEXT_EXPECTED_FRAME = 0;
		this->NEXT_EXPECTED_ACK = 1;
		this->NACK = NACK;
	}
	~ABPSimulator() {
		ES.clear();
	}

	void es_sorted_insert(simEvent event) {
		for (list<simEvent>::iterator next = ES.begin(); next != ES.end();
				++next) {
			if (next->time >= event.time) {
				//printf("inserting before %f",next->time);
				ES.insert(next, event);
				return;
			}
		}
		ES.push_back(event);
	}

	void send_packets() {
		addTimeout();
		send_packet();
		while (this->num_sent_success <= this->num_packets) {
			simEvent latest = getEvent();
			//printEvents();
			//printf("E %u : %f || %f\r\n", latest.type, latest.time, tc);
			if (this->tc < latest.time) {
				this->tc = latest.time;
			}
			//printf("TC:NEW %f\r\n", tc);
			if (latest.type == TIMEOUT) {
				//printf("TIMEOUT SN %d tc %f \r\n", NEXT_EXPECTED_ACK, this->tc);
				fflush(stdout);
				addTimeout(); //Add a new timeout
				send_packet();
			} else if (!latest.error && latest.SN == this->NEXT_EXPECTED_ACK) {
				//printf("OK SN %d tc %f \r\n", NEXT_EXPECTED_ACK, this->tc);
				fflush(stdout);
				this->NEXT_EXPECTED_ACK = (this->NEXT_EXPECTED_ACK + 1) % 2;
				addTimeout(); //Add a new timeout
				send_packet();
			} else {
				if (this->NACK) {
					addTimeout(); //Add a new timeout
					send_packet();
					//retransmit
				}
			}
		}
	}

	void send_packet() {
		double time = (this->tc
				+ (double) (((double) (this->l + this->H)) / ((double) this->C))
				+ this->Tau);
		//printf("SEND TIME: %f %f\r\n", time, this->Tau);
		SEND(time, NEXT_EXPECTED_FRAME);
	}

	void printEvents() {
		printf("\nEVENTS (%d/%d):", num_sent_success, num_packets);
		list<simEvent>::iterator next = ES.begin();
		while (next != ES.end()) {
			printf("(%u:%f:sn:%d:%s) ", next->type, next->time, next->SN,
					(next->type == ACK && next->error) ? "ERR" : "");
			++next;
		}
		printf("\n");
	}

	void purgeTimeouts() {
		ES.remove_if(timeout_event);
	}

	void addTimeout() {
		purgeTimeouts();
		simEvent timeout;
		timeout.type = TIMEOUT;
		timeout.time = (this->tc
				+ (double) (((double) (this->H + this->l)) / this->C)
				+ this->Tau + this->Delta);
		this->es_sorted_insert(timeout);
	}

	simEvent getEvent() {
		simEvent v = ES.front();
		ES.pop_front();
		return v;
	}

	void SEND(double time, int SN) {
		ErrorType rcv_value = this->RECIEVE(time, SN);
		this->tc = time;
		if (rcv_value != LOSS) {
			//NOT LOST - ADD ACK TO ES
			simEvent ack;
			ack.type = ACK;
			ack.SN = NEXT_EXPECTED_FRAME;
			ack.time = this->tc + this->Tau + (((double) this->H) / this->C);
			ack.error = (bool) (rcv_value == ERROR);

			ErrorType ack_sent = channel(this->H);
			if (ack_sent == LOSS) {
				//Ack was lost
				return;
			} else if (ack_sent == ERROR) {
				ack.error = true;
			} else {
				//ack sent ok as is
			}
			this->es_sorted_insert(ack);
		} else {
			//printf("LOSS SN %d tc %f \r\n", SN, this->tc);
			//fflush(stdout);
		}
		return;
	}
	ErrorType channel(int num_bits) {
		//Calculate bits in error, and flag packet
		if (BER == 0) {
			return NONE;
		}
		int BIE = bits_in_error(num_bits);
		// LOSS - 5 or more bit errors to be a lost frame
		//ERROR - 1 to 4 bits in error
		//OK - 0
		if (BIE >= 5) {
			return LOSS;
		} else if (BIE > 1) {
			//ERROR
			return ERROR;
		} else {
			//OK
		}
		return NONE;
	}

	int bits_in_error(int num_bits) {
		/*
		 We expect you to run L iterations.
		 1-> probability BER
		 Count errors for L bits
		 */
		int count = 0;
		for (int i = 0; i < num_bits; i++) {
			count += (uniform_rv() < this->BER) ? 1 : 0;
		}
		return count;
	}

	ErrorType RECIEVE(double tc, int SN) {
		//Calculate bits in error, and flag packet
		ErrorType rcv_status = channel(this->l + this->H);
		// LOSS - 5 or more bit errors to be a lost frame
		//ERROR - 1 to 4 bits in error
		//OK - 0
		if (rcv_status == LOSS) {
			return LOSS;
		} else if (rcv_status == ERROR) {
			return ERROR;
		} else {
			if (SN == NEXT_EXPECTED_FRAME) {
				NEXT_EXPECTED_FRAME = (NEXT_EXPECTED_FRAME + 1) % 2;
				this->num_sent_success++;
				if (num_sent_success == num_packets) {
					total_time = tc;
				}
			}
			//OK
		}
		return NONE;
	}
};

class GBN {
	int H, l, window_size;
	double Delta, C, Tau, BER; //current-time
	list<simEvent> ES;
	list<Packet> BUFFER;
public:
	/////////STATS/////
	double tc;
	double total_time;
	int num_packets;
	int num_sent_success;
	int NEXT_EXPECTED_FRAME;
	int NEXT_EXPECTED_ACK;
	int NEXT_SN;
	bool NACK;
	//////////////////

	GBN(int H, int l, double Delta, double C, double Tau, double BER,
			int num_packets, bool nack, int window_size) {
		this->H = H;
		this->l = l;
		this->Delta = Delta;
		this->C = C;
		this->Tau = Tau;
		this->BER = BER;
		this->num_packets = num_packets;
		this->tc = 0;
		this->window_size = window_size;
		/*
		 * INITIAL VALUES
		 */
		this->num_sent_success = 0;
		this->NEXT_SN = 0;
		this->NEXT_EXPECTED_FRAME = 0;
		this->NACK = nack;
	}
	~GBN() {
		ES.clear();
	}

	void es_sorted_insert(simEvent event) {
		for (list<simEvent>::iterator next = ES.begin(); next != ES.end();
				++next) {
			if (next->time >= event.time) {
//				printf("inserting before %f",next->time);
				ES.insert(next, event);
				return;
			}
		}
		ES.push_back(event);
	}

	bool rotate_buffer(int SN) {
		int position = 1;
		for (list<Packet>::iterator pkt = BUFFER.begin(); pkt != BUFFER.end();
				++pkt) {
			if (pkt->SN == SN) {
				//if element, transmit new packets to fill buffer
				//count position of the element
				for (int i = 0; i < position; i++) {
					BUFFER.pop_front();
					Packet tx;
					this->tc += (((double) (this->l + this->H)
							/ (double) this->C) + this->Tau);
					tx.timeout = this->tc + (double) this->Delta;
					tx.SN = this->NEXT_SN;
					BUFFER.push_back(tx);
					SEND(this->tc, tx.SN);
					this->NEXT_SN = (this->NEXT_SN + 1) % (window_size + 1);
				}
				//purge timeouts
				//set timeout
				addTimeout(BUFFER.front().timeout);
				return true;
			}
			position++;
		}

		return false;
	}

	bool sn_in_buffer(int SN) {
		for (list<Packet>::iterator pkt = BUFFER.begin(); pkt != BUFFER.end();
				++pkt) {
			if (pkt->SN == SN) {
				return true;
			}
		}
		return false;
	}
	void replace_packet(Packet pack) {
		for (list<Packet>::iterator pkt = BUFFER.begin(); pkt != BUFFER.end();
				++pkt) {
			if (pkt->SN == pack.SN) {
				*pkt = pack;
			}
		}
	}

	void retransmit_all() {
		list<Packet> NEWBUFFER;
		for (list<Packet>::iterator pkt = BUFFER.begin(); pkt != BUFFER.end();
				++pkt) {
			this->tc += (((double) (this->l + this->H) / this->C) + this->Tau);
			Packet rx;
			rx.timeout = this->tc + this->Delta;
			rx.SN = pkt->SN;
			NEWBUFFER.push_back(rx);
			SEND(this->tc, rx.SN); //resend packet with this SN
		}
		BUFFER.clear();
		BUFFER = NEWBUFFER;
		addTimeout(BUFFER.front().timeout);
	}

	void send_packets() {
		for (int i = 0; i < window_size; ++i) {
			this->tc += (((double) (this->l + this->H) / this->C) + this->Tau);
			Packet pkt;
			pkt.SN = this->NEXT_SN;
			pkt.timeout = this->tc + this->Delta;
			BUFFER.push_back(pkt);
			SEND(this->tc, pkt.SN);
			this->NEXT_SN = (this->NEXT_SN + 1) % (this->window_size + 1);
		}
		//printf("SET TO SN %d tc %f \r\n", BUFFER.front().SN,
		//		BUFFER.front().timeout);
		addTimeout(BUFFER.front().timeout);
		while (this->num_sent_success <= this->num_packets) {
			simEvent latest = getEvent();
			if (latest.time > this->tc) {
				this->tc = latest.time;
			}
			if (latest.type == TIMEOUT) {
				//printf("TIMEOUT SN %d tc %f \r\n", this->NEXT_SN, this->tc);
				retransmit_all();
			} else {
				int source_sn =
						(latest.SN - 1 < 0) ? window_size : latest.SN - 1;
				if (!latest.error) {
					//check if sn in the buffer, and transmit new packets to fill the buffer
					rotate_buffer(source_sn);
					//printf("OK SN %d tc %f \r\n", latest.SN, this->tc);
				} else {
					//printf("ERR SN %d tc %f \r\n", latest.SN, this->tc);
					//ERROR FRAME
					//IGNORED!
					if (this->NACK) {
						this->tc += ((double) (this->l + this->H)
								/ (double) this->C) + this->Tau;
						Packet pkt;
						pkt.SN = latest.SN;
						pkt.timeout = this->tc + (double) this->Delta;
						replace_packet(pkt);
						SEND(this->tc, latest.SN);
					}
				}
			}
		}
	}

	void purgeTimeouts() {
		ES.remove_if(timeout_event);
	}

	void addTimeout(double time) {
		purgeTimeouts();
		simEvent timeout;
		timeout.type = TIMEOUT;
		timeout.SN = 99;
		timeout.time = time;
		this->es_sorted_insert(timeout);
	}

	simEvent getEvent() {
		//printEvents();
		simEvent v = ES.front();
		ES.pop_front();
		return v;
	}

	void SEND(double time, int SN) {
		//printf("SEND SN %d tc %f \r\n", SN, time);
		ErrorType rcv_value = this->RECIEVE(time, SN);
		if (rcv_value != LOSS) {
			//NOT LOST - ADD ACK TO ES
			simEvent ack;
			ack.type = ACK;
			ack.SN = NEXT_EXPECTED_FRAME;
			ack.time = time + this->Tau + (((double) this->H) / this->C);
			ack.error = (bool) (rcv_value == ERROR);
			ErrorType ack_sent = channel(this->H);
			if (ack_sent == LOSS) {
				//Ack was lost
				return;
			} else if (ack_sent == ERROR) {
				ack.error = true;
			} else {
				//ack sent ok as is
			}
			//printf("ACK (ADDED) SN %d tc %f \r\n", ack.SN, ack.time);
			this->es_sorted_insert(ack);
		} else {
			//printf("LOSS SN %d tc %f \r\n", SN, this->tc);
			//fflush(stdout);
		}
		return;
	}
	void printEvents() {
		printf("\nEVENTS (%d/%d):", num_sent_success, num_packets);
		list<simEvent>::iterator next = ES.begin();
		while (next != ES.end()) {
			printf("(%u:%f:sn:%d:%s) ", next->type, next->time, next->SN,
					(next->type == ACK && next->error) ? "ERR" : "");
			++next;
		}
		printf("\n");
	}
	ErrorType channel(int num_bits) {
		//Calculate bits in error, and flag packet
		if (BER == 0) {
			return NONE;
		}
		int BIE = bits_in_error(num_bits);
		// LOSS - 5 or more bit errors to be a lost frame
		//ERROR - 1 to 4 bits in error
		//OK - 0
		if (BIE >= 5) {
			return LOSS;
		} else if (BIE > 1) {
			//ERROR
			return ERROR;
		} else {
			//OK
		}
		return NONE;
	}

	int bits_in_error(int num_bits) {
		/*
		 We expect you to run L iterations.
		 1-> probability BER
		 Count errors for L bits
		 */
		int count = 0;
		for (int i = 0; i < num_bits; i++) {
			count += (uniform_rv() < this->BER) ? 1 : 0;
		}
		return count;
	}
	ErrorType RECIEVE(double time, int SN) {
		//Calculate bits in error, and flag packet
		ErrorType rcv_status = channel(this->l + this->H);
		if (rcv_status == LOSS) {
			return LOSS;
		} else if (rcv_status == ERROR) {
			return ERROR;
		} else {
			if (sn_in_buffer(SN)) {
				this->num_sent_success++;
				if (this->num_sent_success == this->num_packets) {
					this->total_time = time;
				}
				if(SN == NEXT_EXPECTED_FRAME){
					NEXT_EXPECTED_FRAME = (NEXT_EXPECTED_FRAME + 1)
							% (window_size + 1);
				}
			}
			//OK
		}
		return NONE;
	}
};

}
list<double> question_1(double BER, double Tau, bool nack) {
//	printf("ABP\n");
//	printf("BER: %f\n", BER);
//	printf("H\tL\tDelta\tC\tTau\tBER\t#Packets\tThroughput\n");
	double C = 5000000.0;
	int num_packets = 10000;
	int L = 1500 * 8;
	int H = 54 * 8;
	list<double> results;
	for (double x = 2.5; x < 13; x += 2.5) {
		double Delta = x * Tau;
		Sim::ABPSimulator* sim = new Sim::ABPSimulator(H, L, Delta, C, Tau, BER,
				num_packets, nack);
		sim->send_packets();
		//printf("%d\t%d\t%f\t%d\t%f\t%f\t%d\t", H, L, Delta, C, Tau, BER,
		//num_packets
		//);
		double tput = ((double) num_packets * (double) L)
				/ (double) sim->total_time;
		//printf("%f\n",
		//		tput);
		results.push_back(tput);
		fflush(stdout);
		delete sim;
	}
	return results;
}

void print_data(list<list<double> > data) {
	printf("\n");
	for (int i = 0; i < 5; i++) {
		for (list<list<double> >::iterator ber_data = data.begin();
				ber_data != data.end(); ++ber_data) {
			double tput = *(ber_data->begin());
			ber_data->pop_front();
			printf("%f,", tput);
		}
		printf("\n");
		fflush(stdout);
	}
}

list<double> question_3(double BER, double Tau, int window_size, bool nack) {
//	printf("GBN\n");
//	printf("Buffer Size: %d\n", window_size);
//	printf("BER: %f\n", BER);
//	printf("H\tL\tDelta\tC\tTau\tBER\t#Packets\tThroughput\n");
	double C = 5000000.0;
	int num_packets = 10000;
	int L = 1500 * 8;
	int H = 54 * 8;
	list<double> results;
	for (double x = 2.5; x < 13; x += 2.5) {
		double Delta = x * Tau;
		Sim::GBN* sim = new Sim::GBN(H, L, Delta, C, Tau, BER, num_packets,
				nack, window_size);
		sim->send_packets();
		//printf("%d\t%d\t%f\t%d\t%f\t%f\t%d\t", H, L, Delta, C, Tau, BER,
		//num_packets
		//);
		double tput = ((double) num_packets * (double) L)
				/ (double) sim->total_time;
		//printf("%f\n",
		//		tput);
		results.push_back(tput);
		fflush(stdout);
		delete sim;
	}
	return results;
}

int main(int argc, char* argv[]) {
	srand(time(NULL));
	if (argc == 1) {
		double C = 5000000.0;
		int num_packets = 100;
		int L = 1500 * 8;
		int H = 54 * 8;
		double Tau = 1;
		int Delta = 4 * Tau;
		double BER = 0.0001;
		int window_size = 5;
		Sim::GBN* sim = new Sim::GBN(H, L, Delta, C, Tau, BER, num_packets,
				false, window_size);
		sim->send_packets();
		printf("%f", sim->total_time);
		fflush(stdout);
		return 0;
	}
	list<list<double> > table_data;
	if (*argv[1] == '1') {
		table_data.push_back(question_1(0.0, 0.005, false));
		table_data.push_back(question_1(pow(10, -5), 0.005, false));
		table_data.push_back(question_1(pow(10, -4), 0.005, false));
		table_data.push_back(question_1(0.0, 0.250, false));
		table_data.push_back(question_1(pow(10, -5), 0.250, false));
		table_data.push_back(question_1(pow(10, -4), 0.250, false));
		print_data(table_data);
		table_data.clear();
	}
	if (*argv[1] == '2') {
		table_data.push_back(question_1(0.0, 0.005, true));
		table_data.push_back(question_1(pow(10, -5), 0.005, true));
		table_data.push_back(question_1(pow(10, -4), 0.005, true));
		table_data.push_back(question_1(0.0, 0.250, true));
		table_data.push_back(question_1(pow(10, -5), 0.250, true));
		table_data.push_back(question_1(pow(10, -4), 0.250, true));
		print_data(table_data);
		table_data.clear();
	}
	if (*argv[1] == '3') {
		int N = 4;
		table_data.push_back(question_3(0.0, 0.005, N, false));
		table_data.push_back(question_3(pow(10, -5), 0.005, N, false));
		table_data.push_back(question_3(pow(10, -4), 0.005, N, false));
		table_data.push_back(question_3(0.0, 0.250, 4, false));
		table_data.push_back(question_3(pow(10, -5), 0.250, N, false));
		table_data.push_back(question_3(pow(10, -4), 0.250, N, false));
		print_data(table_data);
		table_data.clear();
	}
	if (*argv[1] == '4') {
		table_data.push_back(question_3(0.0, 0.005, 0, true));
		table_data.push_back(question_3(pow(10, -5), 0.005, 0, true));
		table_data.push_back(question_3(pow(10, -4), 0.005, 0, true));
		table_data.push_back(question_3(0.0, 0.250, 0, false));
		table_data.push_back(question_3(pow(10, -5), 0.250, 0, true));
		table_data.push_back(question_3(pow(10, -4), 0.250, 0, true));
		print_data(table_data);
	}
	return 0;
}
;

