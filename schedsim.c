#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "process.h"
#include "util.h"

/*
 * Assumptions:
 *  - ProcessType has fields: pid, bt, art, wt, tat, pri
 *  - initProc(const char *filename, int *n) returns a malloc'd array of ProcessType
 *    and sets *n to the number of processes.
 *  - printMetrics(ProcessType *plist, int n) prints the per-process and average metrics.
 *
 * This file implements:
 *  - FCFS (arrival-aware; sorts by arrival time)
 *  - Priority (stable behavior by breaking ties using arrival time and pid)
 *  - SRTF (preemptive SJF; checks arrivals at every time unit)
 *  - Round Robin (arrival-aware; advances time to next arrival if CPU idle)
 *
 * It also frees proc_list after each scheduling run to avoid leaks.
 */

/* -----------------------
   Helper qsort comparators
   ----------------------- */

/* FCFS comparator: sort by arrival time, then pid (keeps original input order for ties) */
int compareArrivalThenPid(const void *a, const void *b) {
    const ProcessType *p1 = (const ProcessType *)a;
    const ProcessType *p2 = (const ProcessType *)b;

    if (p1->art < p2->art) return -1;
    if (p1->art > p2->art) return 1;
    /* tie-break with pid so input order is preserved if pid corresponds to input order */
    return p1->pid - p2->pid;
}

/* Priority comparator: primary = priority (lower value = higher priority),
   secondary = arrival time (earlier arrival first),
   tertiary = pid (preserve input order).
   This ensures FCFS among equal priorities. */
int comparePriorityStable(const void *a, const void *b) {
    const ProcessType *p1 = (const ProcessType *)a;
    const ProcessType *p2 = (const ProcessType *)b;

    if (p1->pri < p2->pri) return -1;
    if (p1->pri > p2->pri) return 1;

    if (p1->art < p2->art) return -1;
    if (p1->art > p2->art) return 1;

    return p1->pid - p2->pid;
}

/* -----------------------
   FCFS scheduling
   ----------------------- */

/* Compute waiting times for FCFS when arrival times may be arbitrary.
   We sort by arrival time and then compute service_time cumulatively.
   This implementation assumes we can reorder ready queue by arrival time
   (which FCFS semantics require). */
void findWaitingTimeFCFS(ProcessType plist[], int n) {
    if (n <= 0) return;

    /* Sort by arrival so FCFS processes are in the correct order */
    qsort(plist, n, sizeof(ProcessType), compareArrivalThenPid);

    int service_time = 0;
    for (int i = 0; i < n; i++) {
        /* If the CPU is idle until this process arrives, advance service_time */
        if (service_time < plist[i].art) {
            service_time = plist[i].art;
        }

        plist[i].wt = service_time - plist[i].art;
        if (plist[i].wt < 0) plist[i].wt = 0;

        /* process runs to completion */
        service_time += plist[i].bt;
    }
}

void findavgTimeFCFS(ProcessType plist[], int n) {
    findWaitingTimeFCFS(plist, n);
    for (int i = 0; i < n; i++) {
        plist[i].tat = plist[i].wt + plist[i].bt;
    }
}

/* -----------------------
   Priority scheduling (non-preemptive)
   ----------------------- */

/* Use a stable approach via comparator tie-breakers (priority -> arrival -> pid). */
void findavgTimePriority(ProcessType plist[], int n) {
    if (n <= 0) return;

    qsort(plist, n, sizeof(ProcessType), comparePriorityStable);

    /* After sorting by priority (and tiebreakers), use FCFS semantics */
    /* Reuse the FCFS logic, but since we've already sorted by priority + arrival,
       we can compute waiting times similarly to FCFS but without re-sorting. */
    int service_time = 0;
    for (int i = 0; i < n; i++) {
        if (service_time < plist[i].art)
            service_time = plist[i].art;

        plist[i].wt = service_time - plist[i].art;
        if (plist[i].wt < 0) plist[i].wt = 0;

        service_time += plist[i].bt;
    }

    for (int i = 0; i < n; i++)
        plist[i].tat = plist[i].wt + plist[i].bt;
}

/* -----------------------
   SRTF (preemptive SJF)
   ----------------------- */

/*
 * Preemptive SJF (Shortest Remaining Time First).
 * This algorithm iterates time t = 0,1,2,... until all processes complete.
 * At each time unit it selects the arrived process with the smallest remaining time.
 * It executes that process for 1 time unit (so new arrivals can preempt).
 */
void findWaitingTimeSJF(ProcessType plist[], int n) {
    if (n <= 0) return;

    int *rt = (int *)malloc(sizeof(int) * n);
    if (!rt) {
        fprintf(stderr, "Memory allocation failed in findWaitingTimeSJF\n");
        exit(1);
    }

    for (int i = 0; i < n; i++)
        rt[i] = plist[i].bt;

    int completed = 0;
    int t = 0;

    /* To handle arbitrary arrival times, we do not sort plist here; we check arrival in loop */
    while (completed < n) {
        int idx_shortest = -1;
        int min_rem = INT_MAX;

        /* find arrived process with minimal remaining time */
        for (int i = 0; i < n; i++) {
            if (plist[i].art <= t && rt[i] > 0 && rt[i] < min_rem) {
                min_rem = rt[i];
                idx_shortest = i;
            }
        }

        if (idx_shortest == -1) {
            /* No process has arrived yet or all arrived ones are complete -> advance time to next arrival */
            int earliest_future = INT_MAX;
            for (int i = 0; i < n; i++) {
                if (rt[i] > 0 && plist[i].art > t) {
                    if (plist[i].art < earliest_future)
                        earliest_future = plist[i].art;
                }
            }
            if (earliest_future != INT_MAX)
                t = earliest_future;
            else
                break; /* should not happen but defensive */
            continue;
        }

        /* execute for 1 time unit */
        rt[idx_shortest]--;
        t++;

        /* if finished, mark completion */
        if (rt[idx_shortest] == 0) {
            completed++;
            int finish_time = t;
            plist[idx_shortest].wt = finish_time - plist[idx_shortest].bt - plist[idx_shortest].art;
            if (plist[idx_shortest].wt < 0) plist[idx_shortest].wt = 0;
        }
        /* loop continues; new arrivals at time t may preempt on next step */
    }

    free(rt);
}

/* wrapper to compute tat after waiting times */
void findavgTimeSJF(ProcessType plist[], int n) {
    findWaitingTimeSJF(plist, n);
    for (int i = 0; i < n; i++)
        plist[i].tat = plist[i].wt + plist[i].bt;
}

/* -----------------------
   Round Robin (arrival-aware)
   ----------------------- */

/*
 * Arrival-aware RR:
 * - We maintain rem_bt[] as remaining burst times.
 * - We scan processes in circular order starting from last_index to find the next eligible
 *   (arrived and rem_bt > 0) process to run.
 * - If no process has arrived yet but some processes are still pending (future arrivals),
 *   we advance time to the earliest next arrival.
 * - When a process finishes we compute its waiting time:
 *     wt = finish_time - bt - art
 *
 * This approach avoids assuming all art == 0.
 */
void findWaitingTimeRR(ProcessType plist[], int n, int quantum) {
    if (n <= 0) return;
    if (quantum <= 0) quantum = 1;

    int *rem_bt = (int *)malloc(sizeof(int) * n);
    if (!rem_bt) {
        fprintf(stderr, "Memory allocation failed in findWaitingTimeRR\n");
        exit(1);
    }
    int remaining = n;

    for (int i = 0; i < n; i++) {
        rem_bt[i] = plist[i].bt;
        plist[i].wt = 0; /* initialize */
    }

    int t = 0;
    int last_idx = -1; /* start scanning at 0 on first iteration */

    while (remaining > 0) {
        int found = 0;
        int start = (last_idx + 1) % n;

        /* circular scan once to find a ready process */
        for (int i = 0; i < n; i++) {
            int idx = (start + i) % n;
            if (rem_bt[idx] > 0 && plist[idx].art <= t) {
                /* this process is ready; run it */
                found = 1;
                last_idx = idx;

                if (rem_bt[idx] > quantum) {
                    rem_bt[idx] -= quantum;
                    t += quantum;
                } else {
                    /* last slice for this process */
                    t += rem_bt[idx];
                    rem_bt[idx] = 0;
                    remaining--;

                    plist[idx].wt = t - plist[idx].bt - plist[idx].art;
                    if (plist[idx].wt < 0) plist[idx].wt = 0;
                }

                /* after a run, break so RR fairness (next round) continues scanning from next process */
                break;
            }
        }

        if (!found) {
            /* No ready process found at current time t.
               If there are still unfinished processes, advance time to the earliest arrival
               of the remaining processes (to avoid busy-waiting). */
            int earliest_future = INT_MAX;
            for (int i = 0; i < n; i++) {
                if (rem_bt[i] > 0 && plist[i].art > t) {
                    if (plist[i].art < earliest_future)
                        earliest_future = plist[i].art;
                }
            }

            if (earliest_future == INT_MAX) {
                /* No future arrivals (shouldn't happen if remaining>0), but break defensively */
                break;
            } else {
                t = earliest_future;
            }
        }
    }

    free(rem_bt);
}

void findavgTimeRR(ProcessType plist[], int n, int quantum) {
    findWaitingTimeRR(plist, n, quantum);
    for (int i = 0; i < n; i++)
        plist[i].tat = plist[i].wt + plist[i].bt;
}

/* -----------------------
   Main
   ----------------------- */
int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <inputfile> [quantum]\n", argv[0]);
        return 1;
    }

    int quantum = 2; /* default */
    if (argc >= 3) {
        int q = atoi(argv[2]);
        if (q > 0) quantum = q;
    }

    int n = 0;
    ProcessType *proc_list = NULL;

    /* ------ FCFS ------ */
    proc_list = initProc(argv[1], &n);
    if (!proc_list) {
        fprintf(stderr, "initProc failed or returned NULL for FCFS\n");
        return 1;
    }
    findavgTimeFCFS(proc_list, n);
    printMetrics(proc_list, n);
    free(proc_list);
    proc_list = NULL;

    /* ------ SRTF (preemptive SJF) ------ */
    proc_list = initProc(argv[1], &n);
    if (!proc_list) {
        fprintf(stderr, "initProc failed or returned NULL for SRTF\n");
        return 1;
    }
    findavgTimeSJF(proc_list, n);
    printMetrics(proc_list, n);
    free(proc_list);
    proc_list = NULL;

    /* ------ PRIORITY ------ */
    proc_list = initProc(argv[1], &n);
    if (!proc_list) {
        fprintf(stderr, "initProc failed or returned NULL for Priority\n");
        return 1;
    }
    findavgTimePriority(proc_list, n);
    printMetrics(proc_list, n);
    free(proc_list);
    proc_list = NULL;

    /* ------ ROUND ROBIN ------ */
    proc_list = initProc(argv[1], &n);
    if (!proc_list) {
        fprintf(stderr, "initProc failed or returned NULL for RR\n");
        return 1;
    }
    findavgTimeRR(proc_list, n, quantum);
    printMetrics(proc_list, n);
    free(proc_list);
    proc_list = NULL;

    return 0;
}
