/* =================================================================================
File name:       PID_REG3.H  (IQ version)                    
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type definitions, and 
function prototypes for the PIDREG3.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
------------------------------------------------------------------------------*/
#ifndef __PIDREG3_H__
#define __PIDREG3_H__


typedef struct {

	  	  	  	  _iq  B0;				// Parameter: Proportional gain
	  	  	  	  _iq  B1;				// Variable: Proportional output
	  	  	  	  _iq  B2;				// Variable: Pre-Saturated Integral output

				  _iq  Ref;   			// Input: Reference input
				  _iq  Fdb;   			// Input: Feedback input 
				  _iq  Err_n;			// Variable: Error
				  _iq  Err_n_1;			// Variable: Old_Error
				  _iq  Err_n_2;			// Variable: Oldest_Error

				  _iq  Out;   			// Output: PID output
				  _iq  Out_n_1;   		// Output: Old PID output

				  _iq Ub0;				// Variable: Saturated difference
				  _iq Ub1;			    // Parameter: Integral gain

		 	 	  void  (*calc)();	  	// Pointer to calculation function
				 } PIDREG3;	            

typedef PIDREG3 *PIDREG3_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG3 object.
-----------------------------------------------------------------------------*/                     
#define PIDREG3_DEFAULTS { _IQ(0.125), \
						   _IQ(-0.14), \
						   _IQ(0.03), \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
              			  (void (*)(Uint32))pid_reg3_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG3.C
------------------------------------------------------------------------------*/
void pid_reg3_calc(PIDREG3_handle);

#endif // __PIDREG3_H__

#define PIDREG3_CURRENT  { _IQ(0.0), \
						   _IQ(0.0), \
						   _IQ(0.0), \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
              			  (void (*)(Uint32))pid_reg3_calc }
