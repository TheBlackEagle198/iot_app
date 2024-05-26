#pragma once

#include "CronJob.h"
template <int MAX_SIZE>
class CronJobQueue {
private:
    CronJob* m_jobs[MAX_SIZE];

public:
    CronJobQueue() {
        for (int i = 0; i < MAX_SIZE; i++) {
            this->m_jobs[i] = nullptr;
        }
    }

    void addJob(CronJob* job) {
        for (int i = 0; i < MAX_SIZE; i++) {
            if (this->m_jobs[i] == nullptr) {
                this->m_jobs[i] = job;
                break;
            }
        }
    }

    void runJobs() {
        for (int i = 0; i < MAX_SIZE; i++) {
            if (this->m_jobs[i] != nullptr) {
                this->m_jobs[i]->run();
            }
        }
    }
};