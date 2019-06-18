#ifndef BUFFWR_IMAGE_H
#define BUFFWR_IMAGE_H

#include <buffwrBase.hpp>


class buffwrImage: public buffwrBase
{
public:

    buffwrImage(): _buffer(0)
    {
        _buffer.assign_copy_func(&_cv_Mat_copy_func);
    }
    buffwrImage(size_t buffer_length_in): _buffer(buffer_length_in)
    {
        _buffer.assign_copy_func(&_cv_Mat_copy_func);
    }

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
    static bool _cv_Mat_copy_func(cv::Mat & _target, const cv::Mat & _source){
        // _target = _source.clone();
        _source.copyTo(_target);
        return true;
    }

private:
    // The buffer
    async_buffer< cv::Mat >  _buffer;
};


#endif // BUFFWR_IMAGE_H
