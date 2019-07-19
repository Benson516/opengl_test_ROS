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


    //
    glm::mat3 getRotationMatrix(const glm::mat4 & tf_in);
    glm::mat3 getScewSymmetricMatreix(const glm::vec3 & w_in);
    glm::vec3 getVectorFromScewSymetry(const glm::mat3 & W_in);

protected:
    glm::mat4 tf_input;
    glm::mat4 tf_filtered;

    glm::mat3 R_delta; // filtered --> input


private:

};

#endif // TRANSFORM_FILTER_H
