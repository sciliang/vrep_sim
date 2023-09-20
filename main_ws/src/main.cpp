#include <iostream>
#include <glog/logging.h>
#include "global.hpp"
#include "communication.hpp"
#include "Driver.hpp"
#include "coppeliaSimMsg.hpp"
#include <thread>
#include <signal.h>
using namespace std;

int main(int argc, char *argv[])
{
    CoppeliaSim CoppeliaSim_;
  
    /*---从vrep小车读回 惯导和GPS数据---*/
    thread coppeliasim_read(&CoppeliaSim::CoppeliaSim_run, &CoppeliaSim_);
    thread::id coppeliasim_read_id = coppeliasim_read.get_id();
    cout << "coppeliasim_read thread created! id:" << coppeliasim_read_id;
  
    /*---给vrep下的小车下发速度*/
    thread coppeliasim_send(&CoppeliaSim::SendControlMsg_run, &CoppeliaSim_);
    thread::id coppeliasim_send_id = coppeliasim_send.get_id();
    cout << "coppeliasim_send thread created! id:" << coppeliasim_send_id;
  
    coppeliasim_read.join();
    coppeliasim_send.join();
    return EXIT_SUCCESS;
}