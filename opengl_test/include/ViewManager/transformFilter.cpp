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
