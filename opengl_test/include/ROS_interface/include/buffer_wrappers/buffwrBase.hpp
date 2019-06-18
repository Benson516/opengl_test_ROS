#ifndef BUFFWR_BASE_H
#define BUFFWR_BASE_H


// all_header.h
#include <all_header.h>





class buffwrBase{
public:

    buffwrBase(): _buffer(0)
    {}
    buffwrBase(size_t buffer_length_in): _buffer(buffer_length_in)
    {}

    // Public methods
    //---------------------------------------------------------//
    // Please refers to the doc. below for usage of put_any() and front_any().
    inline virtual bool put_any(boost::any & element_in_ptr, bool is_droping=true, const TIME_STAMP::Time &stamp_in=TIME_STAMP::Time::now(), bool is_no_copying=true){
        return _buffer.put_any(element_in_ptr, is_droping, stamp_in, is_no_copying);
    }
    inline virtual bool front_any(boost::any & content_out_ptr, bool is_poping=false, const TIME_STAMP::Time &stamp_req=TIME_STAMP::Time()){
        return _buffer.front_any(content_out_ptr, is_poping, stamp_req);
    }
    // Put in the &(std::shared_ptr<_T>) or &(_T) as element_in_ptr
    inline virtual bool put_void(void * element_in_ptr, bool is_droping=true, const TIME_STAMP::Time &stamp_in=TIME_STAMP::Time::now(), bool is_shared_ptr=true){  // Exchanging the data, fast
        return _buffer.put_void(element_in_ptr, is_droping, stamp_in, is_shared_ptr);
    }
    // Put in the &(std::shared_ptr<_T>) or &(_T) as content_out_ptr
    inline virtual bool front_void(void * content_out_ptr, bool is_poping=false, const TIME_STAMP::Time &stamp_req=TIME_STAMP::Time(), bool is_shared_ptr=true){  // If is_poping, exchanging the data out, fast; if not is_poping, share the content (Note: this may not be safe!!)
        return _buffer.front_void(content_out_ptr, is_poping, stamp_req, is_shared_ptr);
    }
    inline virtual bool pop(){ return _buffer.pop(); }
    inline virtual TIME_STAMP::Time get_stamp(void){return _buffer.get_stamp();} // Note: use this function right after using any one of the "front" method
    //---------------------------------------------------------//



protected:

private:
    // The buffer
    async_buffer<std::string>  _buffer;
};


/*
// Using put_any(): (The following syntex makes a clear poiter transfer withoud copying or sharring)
//---------------------------------------//
boost::any any_ptr;
{
    std::shared_ptr< _T > _content_ptr = std::make_shared< _T >( _content ); // <-- If we already get the std::shared_ptr, ignore this line
    any_ptr = _content_ptr;
} // <-- Note: the _content_ptr is destroyed when leaveing the scope, thus the use_count for the _ptr in any_ptr is "1" (unique).
buffwr_obj.put_any(any_ptr, true, _time_in, true);
//---------------------------------------//

// Using front_any(): (The following syntex makes a clear poiter transfer withoud copying or sharring)
//---------------------------------------//
std::shared_ptr< _T > _content_ptr;
{
    boost::any any_ptr;
    bool result = buffwr_list[topic_id]->front_any(any_ptr, true, _current_slice_time);
    if (result){
        // _content_ptr = boost::any_cast< std::shared_ptr< cv::Mat > >( any_ptr ); // <-- Not good, this makes a copy
        std::shared_ptr< cv::Mat > *_ptr_ptr = boost::any_cast< std::shared_ptr< cv::Mat > >( &any_ptr );
        _content_ptr = *_ptr_ptr;
    }
} // <-- Note: the any_ptr is destroyed when leaving this scope, thus the use_count for _content_ptr is "1" (unique).
//---------------------------------------//
*/

#endif // BUFFWR_BASE_H
