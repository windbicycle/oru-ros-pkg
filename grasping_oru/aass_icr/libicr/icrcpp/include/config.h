/** \file config.h 
 * \brief Holds the default configuration of the
 * finger used in \ref ICR::FingerParameters
 * 
 */

#ifndef config_h___
#define config_h___

#include "utilities.h"

#define DEFAULT_FORCE_MAGNITUDE                     1
#define DEFAULT_DISC                                8
#define DEFAULT_MU_0                                0.5
#define DEFAULT_MU_T                                0.5
#define DEFAULT_CONTACT_TYPE                        Frictional
#define DEFAULT_CONTACT_MODEL_TYPE                  Single_Point
#define DEFAULT_INCLUSION_RULE_TYPE                 Sphere
#define DEFAULT_INCLUSION_RULE_PARAMETER            10
#define DEFAULT_INCLUSION_RULE_FILTER_INSIDE_POINTS false

#define DIVIDE_OWS_BY_LAMBDA                    

#define MULTITHREAD_ICR_COMPUTATION                 

#endif
