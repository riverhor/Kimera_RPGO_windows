#ifndef KIMERARPGO_MAX_CLIQUE_FINDER_FINDCLIQUE_H_
#define KIMERARPGO_MAX_CLIQUE_FINDER_FINDCLIQUE_H_

#include <stdlib.h>
//#include <sys/time.h>
//#include <unistd.h>
#include <cstddef>
#include <iostream>
#include <vector>

#include "KimeraRPGO/max_clique_finder/graphIO.h"

using namespace std;

#ifdef _DEBUG
static int DEBUGG = 1;
#endif

namespace FMC {


// Function Definitions
bool fexists(const char* filename);
double wtime();
void usage(char* argv0);
int getDegree(vector<int>* ptrVtx, int idx);
void print_max_clique(vector<int>& max_clique_data);

int maxClique(CGraphIO* gio, int l_bound, vector<int>* max_clique_data);
void maxCliqueHelper(CGraphIO* gio,
                     vector<int>* U,
                     int sizeOfClique,
                     int* maxClq,
                     vector<int>* max_clique_data_inter);

int maxCliqueHeu(CGraphIO* gio, vector<int>* max_clique_data);
void maxCliqueHelperHeu(CGraphIO* gio,
                        vector<int>* U,
                        int sizeOfClique,
                        int* maxClq,
                        vector<int>* max_clique_data_inter);

}
#endif
