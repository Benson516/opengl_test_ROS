#include <ROS_ICLU3_v0.hpp>

// Debug
#include <iostream>
// test
#include <FILTER_LIB.h>





int main(int argc, char *argv[])
{
    std::shared_ptr<int> a_ptr;
    a_ptr.reset(new int(5));
    std::cout << "a.use_count() = " << a_ptr.use_count() << "\n";

    boost::any any_a = a_ptr;
    a_ptr.reset(new int(0));
    std::cout << "a.use_count() = " << a_ptr.use_count() << "\n";
    std::cout << "any_a.use_count = " << ( boost::any_cast< std::shared_ptr<int> >(any_a) ).use_count() << "\n";
    a_ptr = boost::any_cast< std::shared_ptr<int> >(any_a);
    std::cout << "a.use_count() = " << a_ptr.use_count() << "\n";


    std::shared_ptr<int> *a_ptr_ptr = boost::any_cast< std::shared_ptr<int> >(&any_a);
    std::cout << "a.use_count() = " << a_ptr.use_count() << "\n";

    //
    std::cout << "any_a = " << *( boost::any_cast< std::shared_ptr<int> >(any_a) ) << "\n";
    std::cout << "a.use_count() = " << a_ptr.use_count() << "\n";
    std::cout << "*a_ptr_ptr = " << *( *a_ptr_ptr ) << "\n";
    std::cout << "a.use_count() = " << a_ptr.use_count() << "\n";


    boost::any any_b = any_a;
    std::cout << "b.use_count() = " << ( boost::any_cast< std::shared_ptr<int> >(any_b) ).use_count() << "\n";
    std::cout << "a.use_count() = " << a_ptr.use_count() << "\n";

    //
    a_ptr.reset(new int(10));
    // std::cout << "b.use_count() = " << ( boost::any_cast< std::shared_ptr<int> >(any_b) ).use_count() << "\n";
    boost::any any_a_ptr = &a_ptr;
    std::cout << "a.use_count() = " << a_ptr.use_count() << "\n";
    // std::cout << "any_a = " << *( boost::any_cast< std::shared_ptr<int> >(any_a_ptr) ) << "\n";
    // std::cout << "a.use_count() = " << a_ptr.use_count() << "\n";

    return 0;
}
