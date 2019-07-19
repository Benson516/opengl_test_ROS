#include "transformFilter.hpp"

TRANSFORM_FILTER::TRANSFORM_FILTER():
    tf_input(1.0f),
    tf_filtered(1.0f)
{

}


void TRANSFORM_FILTER::reset(glm::mat4 tf_in){
    tf_input = tf_in;
    tf_filtered = tf_input;
}

void TRANSFORM_FILTER::setInput(const glm::mat4 & tf_in){
    tf_input = tf_in;
}
glm::mat4 TRANSFORM_FILTER::iterateOnce(){
    tf_filtered = tf_input;
    return tf_filtered;
}
glm::mat4 TRANSFORM_FILTER::getOutput(){
    return tf_filtered;
}




//
glm::mat3 TRANSFORM_FILTER::getRotationMatrix(const glm::mat4 & tf_in){
    glm::mat3 _R;
    for (size_t i=0; i<3; ++i){
        _R[i] = tf_in[i].xyz();
    }
    return _R;
}
glm::mat3 TRANSFORM_FILTER::getScewSymmetricMatreix(const glm::vec3 & w_in){
    glm::mat3 _W(0.0f);
    _W[1][0] = -w_in.z;
    _W[2][0] = w_in.y;
    _W[2][1] = -w_in.x;
    _W[0][1] = w_in.z;
    _W[0][2] = -w_in.y;
    _W[1][2] = w_in.x;
    return _W;
}
glm::vec3 TRANSFORM_FILTER::getVectorFromScewSymetry(const glm::mat3 & W_in){
    return glm::vec3( W_in[1][2], W_in[2][0], W_in[0][1]);
}
