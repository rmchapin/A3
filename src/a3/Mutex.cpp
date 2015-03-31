#include "Mutex.hpp"

Mutex::Mutex() {
	pthread_mutex_init(&_mutex, NULL);
}

void Mutex::lock() {
	pthread_mutex_lock(&_mutex);
}

void Mutex::unlock() {
	pthread_mutex_unlock(&_mutex);
}

