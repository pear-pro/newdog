#ifndef __KINEMATIC__
#define __KINEMATIC__
#include "gait.h"

typedef enum {
    FRONT  = 0,   
    BACK = 1    
} LegSide_t;

void inverseKinematic(Position_HandleTypeDef *hposition);
void inverseKinematic_All(void);
void crawl_inverseKinematic(Position_HandleTypeDef *hposition, LegSide_t leg_side);
void crawl_inverseKinematic_All(void);
void FK(Position_HandleTypeDef *hposition);
void FK_All(void);

#endif

