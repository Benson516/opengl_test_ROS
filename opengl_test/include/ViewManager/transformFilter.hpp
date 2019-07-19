#ifndef TRANSFORM_FILTER_H
#define TRANSFORM_FILTER_H

#include "Common.h"

class TRANSFORM_FILTER{
public:
    TRANSFORM_FILTER();
    void reset(glm::mat4 tf_in=glm::mat4(1.0f) );
    //
    void setInput(const glm::mat4 & tf_in);
    glm::mat4 iterateOnce();
    glm::mat4 getOutput();

protected:
    glm::mat4 tf_input;
    glm::mat4 tf_filtered;


private:

};

#endif // TRANSFORM_FILTER_H
