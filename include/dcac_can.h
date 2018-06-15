void Init_CAN_Proj(void);
void CAN_communication_RX(void);
void CAN_communication_TX(void);
void CAN_communication_TX2(void);
unsigned int flipbyte(unsigned int x);

#define CAN_ADRESS 0x190 
#define PHASE_ADRESS 0x0
//#define PHASE_ADRESS 0x1
//#define PHASE_ADRESS 0x2

extern struct CANBus_control_str CANBus_control;
extern union CAN_MESSAGE_reg can_message;
extern struct block_control_str block_control;

struct CANBus_control_str {
	unsigned int master;
	unsigned int lost_master;
	unsigned int refresh_timer;
	unsigned int init_timer;
	unsigned int test;
	unsigned int command_type;
	unsigned int command_data;
	unsigned int tx_flag;
	unsigned int adress;
	unsigned int number_answer;
};



struct byte_array_str
{
	unsigned int byte1:8;
	unsigned int byte0:8;
	unsigned int byte3:8;
	unsigned int byte2:8;
	unsigned int byte5:8;
	unsigned int byte4:8;
	unsigned int byte7:8;
	unsigned int byte6:8;
} ;
struct word_array_str
{
	unsigned int word0;
	unsigned int word1;
	unsigned int word2;
	unsigned int word3;
} ;
union CAN_MESSAGE_reg {
	struct word_array_str word;
	struct byte_array_str byte;
};

struct block_control_str
{
	unsigned int onoff:1;

};


struct block_status_str
{
	unsigned int input_voltage;
	unsigned int input_current;
	unsigned int output_current;
	unsigned int flags;
};

struct pe_status_str
{
	struct block_status_str phase_a;
	struct block_status_str phase_b;
	struct block_status_str phase_c;
};
//EOF
