/*
 * Copyright - Bacher Aaron 2022
 *
 * Simple implementation for ringbuffer / circular buffer
 */

#ifndef RINGBUFFER
#define RINGBUFFER

#include <iterator>

#include <cassert>
#define assertm(exp, msg) assert(((void)msg, exp))

/*
 * Rinbuffer Class
 * Class implements ringbuffer for arbitrary size and type
 */
template <class T>
class Ringbuffer {

public:

    /*
     * Constructor
     * initializes some variables
     * checks that size is >= 1, else Ringbuffer makes no sense
     */
    Ringbuffer(const int size)
            :size_(size)
    {
        assertm(size_ >= 1, "Ringbuffer must have min size=1");

        buffer_ = new T[size_];
        indexReading_ = 0;
        indexWriting_ = 0;
    }

    /*
     * Destructor
     * deallocate buffer array
     */
    ~Ringbuffer(){
        delete[] buffer_;
    }

    /*
     * Method to add a new item to buffer
     * returns true if required space is available, else returns false
     */
    bool add(T newItem){
        if(getUsedSize() == size_){
            std::cerr << "ERROR: Ringbuffer full!" << std::endl;
            return false;;
        }
        std::cout << "Trying to add item at buffer[" << indexWriting_ << "]" << std::endl;
        buffer_[indexWriting_++] = newItem;
        std::cout << "added item" << std::endl;
        return true;
    }

    /*
     * Method to remove item at reading positon
     * removing an item simply means to increase reading index so this position can be overwritten afterwards again
     * Does not return that item
     */
    void pop(){
        // increase reading index only if it isn't writing index (else it would overtake it)
        if (indexReading_ != indexWriting_){
            ++indexReading_;
        }
    }

    /*
     * Method to get item from buffer, item remains in buffer
     * User can specify offset, so offset==1 will return second but next item
     */
    T get(unsigned int offset = 0) {
        if(indexWriting_ == indexReading_){
            std::cerr << "ERROR: Ringbuffer empty!" << std::endl;
        }
        unsigned int index = getNextIndex(offset);
        std::cout << "Trying to get item at buffer[" << index << "]" << std::endl;
        if(index == size_ || index == -1){
            std::cout << "ERROR" << std::endl;
            exit(1);
        }
        return buffer_[index];
    }

    /*
     * Method to get amount of stored items in buffer
     */
    unsigned int getUsedSize(){
        if (indexReading_ < indexWriting_){
            return indexWriting_ - indexReading_;
        }
        else if (indexWriting_ < indexReading_){
            return indexWriting_ - indexReading_ + size_;
        }
        else {
            return 0;
        }
    }


private:

    /*
     * Method to get new index within buffersize
     * Returns -1 if index points to undefined buffer space
     */
    unsigned int getNextIndex(unsigned int offset = 0){
        unsigned int index = (indexReading_ + offset) % size_;

        // check if index is between reading and writing index, else buffer at desired location is undefined
        if(indexReading_ < indexWriting_){
            if(index >= indexReading_ && index < indexWriting_) return index;
            else return -1;
        }
        else{   // writing index is coming "from behind"
            if(index >= indexReading_ || index < indexWriting_) return index;
            else return -1;
        }
    }

private:

    unsigned int size_;

    // storage room
    T* buffer_;

    unsigned int indexReading_;
    unsigned int indexWriting_;

};

#endif
