#ifndef THREAD_SAFETY_CLASSES_H
#define THREAD_SAFETY_CLASSES_H

#include <STM32FreeRTOS.h>
#include <atomic>



class FreeRTOSlock {
    public:
        FreeRTOSlock(SemaphoreHandle_t sh) : sh_int{sh} {
            xSemaphoreTake(sh, portMAX_DELAY);
        }
        ~FreeRTOSlock() {
            xSemaphoreGive(sh_int);
        }
    private:
        SemaphoreHandle_t sh_int;
};

class ThreadSafeBoundedInteger {
    std::atomic<int> number{0};
    const int max;
    const int min;

    public:
        ThreadSafeBoundedInteger(int initialValue) 
            : number(initialValue), min(-2147483648), max(2147483647) {}

        ThreadSafeBoundedInteger(int initialValue, int min_, int max_) 
            : number(initialValue), min(min_), max(max_) {}

        int get() const {
            return number.load(std::memory_order_relaxed);
        }

        bool add(int delta) { // returns false if it exceeds min/max limits
            int oldNumber = number.load(std::memory_order_relaxed);
            int newNumber;
            bool success = true;

            do {
                newNumber = oldNumber + delta;

                if (newNumber > max) {
                    newNumber = max;
                    success = false;
                } else if (newNumber < min) {
                    newNumber = min;
                    success = false;
                }
            } while (!number.compare_exchange_weak(oldNumber, newNumber, std::memory_order_relaxed));

            return success;
        }

        bool increment() {
            return add(1);
        }

        bool decrement() {
            return add(-1);
        }

        bool assign(int newValue) {
            bool success = true;
            if (newValue > max) {
                success = false;
                newValue = max;
            } else if (newValue < min) {
                newValue = min;
                success = false;
            }
            number.store(newValue, std::memory_order_relaxed);
            return success;
        }
};

class ThreadSafeBoolean {
    std::atomic<bool> value;

public:
    ThreadSafeBoolean(bool initialValue = false) : value(initialValue) {}

    bool get() const {
        return value.load(std::memory_order_acquire);
    }

    void set(bool newValue) {
        value.store(newValue, std::memory_order_release);
    }

    void toggle() {
        value.exchange(!get(), std::memory_order_acq_rel);
    }
};



#endif