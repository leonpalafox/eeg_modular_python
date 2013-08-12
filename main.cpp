/********************************************//**
 Main File for the EEG Demo running with separated modules for
 Decoding: Run in Python
 Plotting: Run by graphic_module
 3D Environment: Run using python and C++
 This Core Function is in charge of getting the EEG signals from the
 Virtual Machine Running the capture.
 Using those signals, it sends messages regarding the current target
 current price and next target.



 To send the signals, we use the ZMQ library that to create a virtual network
 in the same computer.

 @author Mosalam, Leon Palafox
 @version 0.0.1
 ***********************************************/
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <thread>
#include <fstream>
#include <sstream>

#include <vector>
#include <iomanip> // setprecision
#include <chrono> // timing
#include <sys/resource.h>

#include <SDL/SDL.h>
//#include <algorithm>
#include <zmq.hpp>

//These libraries are cutsom libraries written by Mosalam and Leon

#include "utils.h"
#include "gnuplot.h" //In charge of outputting the power in real time
#include "eeg_receiver.h" //In charge of receiving the messages of the EEG
#include "fft.h"//In charge of transforming raw EEG in power
#include "statescoremachine.h" //In charge of keeping up the score and current state.


using namespace std;
using namespace chrono;
using namespace zmq;

int kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

int main()
{
    //This socket connects with the graphics module
    context_t context_py(1);
    socket_t publisher(context_py, ZMQ_PUB);
    publisher.bind("tcp://127.0.0.1:50000");

//    socket_t subscriber(context_py, ZMQ_SUB);
    //publisher.setsockopt(ZMQ_PUB,NULL,0);
//    subscriber.connect("tcp://127.0.0.1:50000");
//    subscriber.setsockopt(ZMQ_SUBSCRIBE, NULL, 0);
//    stringstream message;

    // change the proiority and scheduler of process
    /*struct sched_param param;
    param.sched_priority = 99;
    if (sched_setscheduler(0, SCHED_FIFO, & param) != 0) {
      perror("sched_setscheduler");
      exit(EXIT_FAILURE);
    }*/

    // data to be received
    double time;
    size_t numChannels = 65;
    size_t numChannelsWOT = numChannels - 1; //#Channels without the timer
    size_t numScans = 32; //# of scans done by the gTec device
    size_t dataSize = numChannels * numScans;
    size_t laplaSize = 8;
    size_t PlotSize = 3;
    size_t sampling_frq = 4800; // Sampling Frequency of the device
    size_t dft_points = 1024; // #of points to do the DFT 1024
    size_t dataSize_powers = dft_points/2+1; //#of points after doing the dft
    float channels[dataSize];
    
    EegReceiver eeg(numChannels, numScans);

    // save received data into file
    FILE* pFile;
    pFile = fopen("raw_data", "wb");
    setvbuf (pFile, NULL, _IOFBF, dataSize*sizeof(float));

    /*This chunk was from the new format file*/
    //This is for saving the pwoers in file
    FILE* pFile_powers;
    pFile_powers = fopen("raw_data_powers", "wb");
    setvbuf (pFile_powers, NULL, _IOFBF, dataSize_powers*sizeof(float));
    /*This chunk was from the new format file*/
    //Frequency bin width = 2400/512 = 4.6
    //512 comes from sampling frequency
    FILE* laplaFile;
    laplaFile = fopen("lapla_data_powers", "wb");
    setvbuf (laplaFile, NULL, _IOFBF, laplaSize*sizeof(float));
    FILE* PltFile;
    PltFile = fopen("plot_data_powers", "wb");
    setvbuf (PltFile, NULL, _IOFBF, PlotSize*sizeof(float));
    

  cout<<"l0"<<endl;
    Fft<double> fft(dft_points, Fft<double>::windowFunc::NONE, sampling_frq, numChannelsWOT);
  cout<<"l1"<<endl;
    vector<vector<double> > powers(numChannelsWOT);
    vector<vector<double> > phases(numChannelsWOT);
    for (size_t i=0; i<numChannelsWOT; i++) {
      powers[i].resize(dft_points/2+1);
      phases[i].resize(dft_points/2+1);
    }

    vector<double> point(numChannelsWOT);
    cout<<"l0"<<endl;


    struct timespec requestStart, requestEnd;

    // alpha value in first order autoregressive
    float alpha = 0.97;

    // signals gui window to close
    bool exit = false;

    // the ball/cursor whose vertical position(redRect.y) is controlled by EEG

    float reward_money=0.0; //Money Reward to be displayed in the screen
    const size_t numChannelsPlt = 2;
    vector<float> pwrPlt(numChannelsPlt);
    float powersSave[numChannelsPlt+1];
    vector<int> largeLaplacianIdx{27,28,29,30,31,36,37,38,39,40};
    float largeLaplacian[laplaSize];
    vector<string> legends;
    legends.push_back(string("ch 1"));
    legends.push_back(string("ch 2"));
    //legends.push_back(string("ch 3"));
    //legends.push_back(string("ch 4"));
    // gnuplot window to plot live data


    GnuPlot gnuplot(legends);

    // * SDL - graphics
    // register exit function of SDL to release taken memory gracefully
    // title of window
    SDL_WM_SetCaption("Uruk - NSPLab", NULL);
    // create window of 800 by 480 pixels
    // dialog box to modify alpha

    thread gui(runGUI, &alpha, &exit);
    // receive EEG signal using ZMQ

    //relax_csv<<baselineSamples<<endl;
    //left_relax_csv<<baselineSamples_left<<endl;
    //right_relax_csv<<baselineSamples_right<<endl;

    int loop = 0;
    //StateMachine stateMachine(0.4, 5.0, 5.0, 425);    //stateMachine arguments are (lambda, maxtime, holdtime, baseupperbound, baselowerbound)
    float alpha_1=1.5;
    float alpha_2=1.0;
    float beta_1=525;
    float beta_2=325;
    float power_threshold;
    power_threshold = (425*(alpha_1-alpha_2)-alpha_1*beta_2+alpha_2*beta_1)/(beta_1-beta_2);
    float target = 0;
    StateScoreMachine stateMachine(alpha_1);
    stateMachine.setThreshold(power_threshold);
    //sleep(5);
    //625 and 425 are the graphical lower and upper bounds respectively, if the boxes are shifted
    //these numbers have to be changed
    //*********************************************************************
    //*********************************************************************
    //** Experiment
    //In the experiment, we have already calculated the upper and lower bounds
    //for the baseline.
    //*********************************************************************
    //*********************************************************************
    // main loop
    int samples_count = 0;
     //counter holds the register to control how many samples to wait until calculating a new spectrum
      for(; !kbhit();){
         //cout<<"Canda"<<endl;

         eeg.receive(time, channels);
         //cout<<fixed<<setprecision(9)<<time<<endl;
         //cout<<channels[27]<<endl;

        // receive EEG
         for (size_t i=0; i<numScans; i++) {
           for (size_t ch=0; ch<numChannelsWOT; ch++) {
             point[ch] = channels[i*numChannels+ch]; //recording only the last scan of the 32
           }
           // large laplacians
           //Electrodes 27,28, 29, 30, 31,32, 37, 38, 39, 40, 41
           int newIdx;
           float sumNeighb = 0;
           for (int largeIdx = 0; largeIdx<laplaSize; largeIdx++)
           {
               newIdx = largeLaplacianIdx[largeIdx]; //This is the index of the electrode
               sumNeighb = channels[newIdx+2] + channels[newIdx-2] + channels[newIdx + 14] + channels[newIdx - 14];
               largeLaplacian[largeIdx] = channels[newIdx]-0.25 * sumNeighb;
           }
           fwrite(largeLaplacian, sizeof(float), laplaSize, laplaFile);
           point[62] = channels[27] - 0.25 * (channels[25]+channels[9]+channels[29]+channels[45]);
           point[63] = channels[31] - 0.25 * (channels[29]+channels[13]+channels[33]+channels[49]);
           fft.AddPoints(point);
         }

         fwrite(channels, sizeof(float), dataSize, pFile);
         loop += 1;
         //every ~66ms

         if (loop > 10) {
             // 10 * number of scans * 1/sampling rate
             // 10 * 32 * 1/4800 ~ 66 ms
           clock_gettime(CLOCK_REALTIME, &requestStart);
           if (fft.Process()) {
             fft.GetPower(powers);
             //pwrPlt[0] = (alpha) * pwrPlt[0] + (1.0 - alpha) * (powers[27][3] - 0.25 * (powers[25][3]+powers[9][3]+powers[29][3]+powers[45][3]));
             //pwrPlt[1] = (alpha) * pwrPlt[1] + (1.0 - alpha) * (powers[31][3] - 0.25 * (powers[29][3]+powers[13][3]+powers[33][3]+powers[49][3]));
             pwrPlt[0] = (alpha) * pwrPlt[0] + (1.0 - alpha) * powers[62][4];
             pwrPlt[1] = (alpha) * pwrPlt[1] + (1.0 - alpha) * powers[63][4];
             powersSave[0] = pwrPlt[0];
             powersSave[1] = pwrPlt[1];
             powersSave[2] = target;
             cout<<powersSave[0]<<endl;
             cout<<"Samples: "<<samples_count<<endl;
             samples_count++;

             fwrite(&powersSave[0], sizeof(float), PlotSize, PltFile);
             //fwrite(&powers[0], sizeof(double), dataSize_powers, pFile_powers);
             for (size_t power_idx = 0; power_idx<numChannelsWOT; power_idx++)
             {
                 fwrite(&(powers[power_idx][0]), sizeof(double), dataSize_powers, pFile_powers);
             }
             float mean_power = pwrPlt[0];
             float message_array[3];
             stateMachine.CalculateChange(mean_power);
             target = stateMachine.getState();
             reward_money = stateMachine.getReward();
             cout<<"Mean Power is: "<<mean_power<<endl;
             cout<<"Target is: "<<target<<endl;
             cout<<"Reward is: "<<reward_money<<endl;
             message_array[0] = mean_power;
             message_array[1] = target;
             message_array[2] = reward_money;
             zmq:message_t control_signal(3*sizeof(float));
             memcpy(control_signal.data(), &(message_array[0]), 3*sizeof(float));
             publisher.send(control_signal);

                    //- left_av_mu_power;
             // publish computed powers using ZMQ
             stringstream message;
             message<<pwrPlt[2]<<",";
             message<<pwrPlt[3]<<",";
//             message<< mean_power <<",";
//             message<< mean_power <<",";
//             message<<endl;
//             cout << message <<endl;
//             zmq::message_t zmq_message(message.str().length());
//             memcpy((char *) zmq_message.data(), message.str().c_str(), message.str().length());
//             publisher.send(zmq_message);
//             cout<<"You passed the publisher"<<endl;
//             // receive ball pos from python
//             zmq::message_t ball_msg;
//             subscriber.recv(&ball_msg);
//             cout<<"You passed the subscriber"<<endl;
//             // convert reveived data into c++ string/sstream
//             string feat_str(((char *)ball_msg.data()));
//             stringstream ss;
//             ss.str(feat_str);
//             cout<<"You passed the string"<<endl;
//             float x=0;
//             ss>>x;
//             cout<<"Python sent: "<<x<<endl;



             //cb_power.put(mean_power);

            // update plot



            //vec power_vec;
            //cb_power.peek(power_vec);
            //mean_power = mean(power_vec);
            //Given the current position and the original settings, returns the next target
            //target = stateMachine.UpdateState(redRect.y);
            ostringstream iss;
            iss<<"echo "<<(unsigned char)('b'+target)<<" > /dev/ttyACM0";
            string ttyCmd = iss.str();
            cout<<"ttyCmd "<<ttyCmd<<endl;
            system(ttyCmd.c_str());
            //cout<<"x: "<<redRect.x<<endl;

            // SDL section
            //In charge of controlling the vertical boxes shape color and changes
            //DrawGraphics_Augmented(screen,target,reward_money,&redRect);

            // write into csv
            //This routine pushes all the available information in a csv file named 'data.csv'
            //The Format is:
            //|Target|Y position of cursor|Left average power|right average power| channel1|.....|channel64|time|
            //The location is in the same directory as the application
           }
           clock_gettime(CLOCK_REALTIME, &requestEnd);
           double accum = ( requestEnd.tv_sec - requestStart.tv_sec )
             + ( requestEnd.tv_nsec - requestStart.tv_nsec )
             / 1E9;
           cout<<"t: "<<fixed<<setprecision(12)<<accum<<endl;
           loop = 0;
           gnuplot.Plot(pwrPlt);
         }
       }

      fclose(PltFile);
      fclose(pFile_powers);
      fclose(pFile);
      fclose(laplaFile);
      //gui.join();

    return 0;
}

