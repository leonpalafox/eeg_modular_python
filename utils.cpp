#include "utils.h"

#include <Fl/Fl.H>
#include <Fl/Fl_Window.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Output.H>

#include <SDL/SDL_gfxPrimitives.h>
#include "SDL/SDL_ttf.h"
#include "SDL/SDL.h"
#include "SDL/SDL_image.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>
#include <unistd.h>


using namespace std;

//
static void close_cb(Fl_Widget*, void *w) {
  bool* exit = (bool*)w;
  (*exit) = true;
  cout<<"exit        "<<endl;
}

void get_moving_average(vector<vector<double> > &powers, vector<vector<double> > &powers_avg, float alpha)
{
    for (size_t chan_idx = 0; chan_idx<powers_avg.size(); chan_idx++)
    {
        for (size_t freq_idx = 0; freq_idx<powers_avg[0].size(); freq_idx++)
        {
            powers_avg[chan_idx][freq_idx] = alpha * powers_avg[chan_idx][freq_idx] + (1 - alpha)*powers[chan_idx][freq_idx];
        }
    }

}


void calculate_large_laplacian(vector<double> &input_channels, vector<double> &laplacian_channels)
/*************************************************************************
 *This function calculates the large laplacian for each and every one of the channels
 *many parts of the large laplacian are hard coded, since the topography is not common
 *This library is heavily hardcoded, do not modify since is unique for large laplacian.
 *Input:
 *input_channels: is a float array that contains the eeg raw data
 *output_channels: is a float array that will contain the laplacian
 *******************************************************************************/
{
    //First we solve the indexes that havea consisten toography
    vector<int> largeLaplacianIdx{27,28,29,30,31,36,37,38,39,40};
    int newIdx;
    size_t laplaSize = 8;
    float sumNeighb = 0;
    for (int largeIdx = 0; largeIdx<laplaSize; largeIdx++)
    {
        newIdx = largeLaplacianIdx[largeIdx]; //This is the index of the electrode
        sumNeighb = input_channels[newIdx+2] + input_channels[newIdx-2] + input_channels[newIdx + 14] + input_channels[newIdx - 14];
        laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;
    }
    //Hard coded large laplacian values
    //0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26
    //32,33,34,35,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,
    //62,63,64,65,66

    newIdx = 0;
    sumNeighb = input_channels[7] + input_channels[2] + input_channels[10];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 1;
    sumNeighb = input_channels[3] + input_channels[6] + input_channels[11];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 2;
    sumNeighb = input_channels[0] + input_channels[15] + input_channels[12];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 3;
    sumNeighb = input_channels[16] + input_channels[1] + input_channels[10];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 4;
    sumNeighb = input_channels[7] + input_channels[5] + input_channels[18];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 5;
    sumNeighb = input_channels[4] + input_channels[22] + input_channels[15];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 6;
    sumNeighb = input_channels[1] + input_channels[24] + input_channels[13];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 7; //8
    sumNeighb = input_channels[25] + input_channels[0] + input_channels[11];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 8; //9
    sumNeighb = input_channels[26] + input_channels[10] + input_channels[0];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 9; //10
    sumNeighb = input_channels[27] + input_channels[7] + input_channels[11] + input_channels[0];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 10; //11
    sumNeighb = input_channels[28] + input_channels[8] + input_channels[12] + input_channels[0];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 11; //12
    sumNeighb = input_channels[29] + input_channels[9] + input_channels[13] + input_channels[1];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 12; //13
    sumNeighb = input_channels[30] + input_channels[10] + input_channels[14] + input_channels[1];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 13; //14
    sumNeighb = input_channels[31] + input_channels[11] + input_channels[15] + input_channels[1];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 14; //15
    sumNeighb = input_channels[32] + input_channels[12] + input_channels[2];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 15; //16
    sumNeighb = input_channels[33] + input_channels[13] + input_channels[2];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 16; //17
    sumNeighb = input_channels[34] + input_channels[3] + input_channels[18];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 17; //18
    sumNeighb = input_channels[35] + input_channels[3] + input_channels[19];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 18; //19
    sumNeighb = input_channels[36] + input_channels[16] + input_channels[20] + input_channels[4];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;


    newIdx = 19; //20 <----
    sumNeighb = input_channels[37] + input_channels[17] + input_channels[21];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 20; //21
    sumNeighb = input_channels[38] + input_channels[18] + input_channels[22];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 21; //22
    sumNeighb = input_channels[39] + input_channels[19] + input_channels[23];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 22; //23
    sumNeighb = input_channels[40] + input_channels[20] + input_channels[24] + input_channels[5];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 23; //24
    sumNeighb = input_channels[41] + input_channels[21] + input_channels[5];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;


    newIdx = 24; //25
    sumNeighb = input_channels[42] + input_channels[22] + input_channels[6];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 25; //26
    sumNeighb = input_channels[43] + input_channels[7] + input_channels[27];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 26; //27
    sumNeighb = input_channels[44] + input_channels[8] + input_channels[28];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 32; //33
    sumNeighb = input_channels[50] + input_channels[14] + input_channels[30];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 33; //34
    sumNeighb = input_channels[51] + input_channels[15] + input_channels[31];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;


    newIdx = 34; //35
    sumNeighb = input_channels[52] + input_channels[16] + input_channels[36];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 35; //36
    sumNeighb = input_channels[52] + input_channels[17] + input_channels[37];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 41; //42
    sumNeighb = input_channels[56] + input_channels[39] + input_channels[23];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 42; //43
    sumNeighb = input_channels[56] + input_channels[40] + input_channels[24];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 43; //44
    sumNeighb = input_channels[57] + input_channels[25] + input_channels[45];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 44; //45
    sumNeighb = input_channels[57] + input_channels[26] + input_channels[46];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 45; //46
    sumNeighb = input_channels[57] + input_channels[27] + input_channels[47] + input_channels[43];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 46; //47
    sumNeighb = input_channels[58] + input_channels[28] + input_channels[48] + input_channels[44];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 47; //48
    sumNeighb = input_channels[58] + input_channels[29] + input_channels[49] + input_channels[45];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 48; //49
    sumNeighb = input_channels[58] + input_channels[30] + input_channels[50] + input_channels[46];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;


    newIdx = 49; //50 <<<<<<<
    sumNeighb = input_channels[59] + input_channels[31] + input_channels[51] + input_channels[47];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 50; //51
    sumNeighb = input_channels[59] + input_channels[32] + input_channels[48];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 51; //52
    sumNeighb = input_channels[59] + input_channels[33] + input_channels[49];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 52; //53
    sumNeighb = input_channels[58] + input_channels[34] + input_channels[54];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 53; //54<<<<<<<<<<<<<<<<<<<<<<
    sumNeighb = input_channels[36] + input_channels[55];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 54; //55
    sumNeighb = input_channels[38] + input_channels[52] + input_channels[56];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 55; //56
    sumNeighb = input_channels[40] + input_channels[53];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 56; //57
    sumNeighb = input_channels[42] + input_channels[58] + input_channels[48];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;


    newIdx = 57; //58
    sumNeighb = input_channels[43] + input_channels[59] + input_channels[46];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 58; //59
    sumNeighb = input_channels[52] + input_channels[47] + input_channels[56];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 59; //60
    sumNeighb = input_channels[57] + input_channels[47] + input_channels[51];
    laplacian_channels[newIdx] = input_channels[newIdx]-0.25 * sumNeighb;

    newIdx = 60; //61
    laplacian_channels[newIdx] = input_channels[newIdx];

    newIdx = 61; //62
    laplacian_channels[newIdx] = input_channels[newIdx];

    newIdx = 62; //63
    laplacian_channels[newIdx] = input_channels[newIdx];

    newIdx = 63; //64
    laplacian_channels[newIdx] = input_channels[newIdx];


}

void runGUI(float* alpha, bool* exit) {
    char buffer[20]="0.9995";
    char buffer2[20]="0.1";
    //cout<<"Que uvas"<<endl;
    Fl_Window* w = new Fl_Window(1800, 100, 330, 190, "Uruk - NSPLab");
    Fl_Button ok(110, 130, 100, 35, "Update");
    Fl_Input input(60, 40, 250, 25,"Alpha:");
    input.value(buffer);
    w->end();
    w->show();
    w->user_data(exit);
    w->callback((Fl_Callback*)close_cb, exit);

    while (!(*exit)) {
      Fl::wait();
      Fl_Widget *o;
      while (o = Fl::readqueue()) {
        if (o == &ok) {
            strcpy(buffer, input.value());
            (*alpha) = ::atof(buffer);
        }
      }
    }
}

void fill_circle(SDL_Surface *surface, int cx, int cy, int radius, Uint32 pixel) {
    static const int BPP = 4;

    double r = (double)radius;

    for (double dy = 1; dy <= r; dy += 1.0)
    {
        double dx = floor(sqrt((2.0 * r * dy) - (dy * dy)));
        int x = cx - dx;

        // Grab a pointer to the left-most pixel for each half of the circle
        Uint8 *target_pixel_a = (Uint8 *)surface->pixels + ((int)(cy + r - dy)) * surface->pitch + x * BPP;
        Uint8 *target_pixel_b = (Uint8 *)surface->pixels + ((int)(cy - r + dy)) * surface->pitch + x * BPP;

        for (; x <= cx + dx; x++)
        {
            *(Uint32 *)target_pixel_a = pixel;
            *(Uint32 *)target_pixel_b = pixel;
            target_pixel_a += BPP;
            target_pixel_b += BPP;
        }
    }
}

void DrawGraphics(SDL_Surface *screen, int state, SDL_Rect* redRect) {
    SDL_FillRect(screen , NULL , 0x221122);

/*    SDL_Rect workRect;
    workRect.y = 90;
    workRect.x = 365;
    workRect.w = 70;
    workRect.h = 310;

    SDL_Rect workRect2;
    workRect2.y = 95;
    workRect2.x = 370;
    workRect2.w = 60;
    workRect2.h = 300;

    SDL_Rect hRect;
    hRect.y = 245;
    hRect.x = 350;
    hRect.w = 100;
    hRect.h = 2;*/
    int u_rect_x1=885, u_rect_y1=225, u_rect_x2=1085, u_rect_y2=425;
    int m_rect_x1=885, m_rect_y1=425, m_rect_x2=1085, m_rect_y2=625;

    //SDL_FillRect(screen , &workRect , SDL_MapRGB(screen->format , 200 , 200 , 200 ) );
    //SDL_FillRect(screen , &workRect2 , SDL_MapRGB(screen->format , 34 , 17 , 34 ) );
    //SDL_FillRect(screen , &hRect , SDL_MapRGB(screen->format , 34 , 100 , 34 ) );

    switch(state) {
    case 0:
        roundedRectangleColor(screen, m_rect_x1, m_rect_y1, m_rect_x2,
                              m_rect_y2, 20, SDL_MapRGB(screen->format , 200 , 250 , 250 ));
        rectangleColor(screen, u_rect_x1, u_rect_y1, u_rect_x2, 
                       u_rect_y2, SDL_MapRGB(screen->format , 200 , 200 , 200 ));
        break;

    case 1:
        roundedRectangleColor(screen,u_rect_x1, u_rect_y1, u_rect_x2, 
                              u_rect_y2, 20, SDL_MapRGB(screen->format , 200 , 250 , 250 ));
        rectangleColor(screen, m_rect_x1, m_rect_y1, m_rect_x2,
                       m_rect_y2, SDL_MapRGB(screen->format , 200 , 200 , 200 ));
        break;
    }

    SDL_FillRect(screen , redRect , SDL_MapRGB(screen->format , 200 , 20 , 200 ) );

    SDL_Flip(screen);
}
void DrawGraphics_Augmented(SDL_Surface *screen,int state,float pasive_counter, SDL_Rect* redRect) {
    SDL_FillRect(screen , NULL , 0x221122);

/*    SDL_Rect workRect;
    workRect.y = 90;
    workRect.x = 365;
    workRect.w = 70;
    workRect.h = 310;

    SDL_Rect workRect2;
    workRect2.y = 95;
    workRect2.x = 370;
    workRect2.w = 60;
    workRect2.h = 300;
    SDL_Rect hRect;
    hRect.y = 245;
    hRect.x = 350;
    hRect.w = 100;
    hRect.h = 2;*/
    SDL_Rect offset;
    offset.x=1200;
    offset.y=225;

    int price_counter=pasive_counter;
    stringstream ss;
    int u_rect_x1=885, u_rect_y1=225, u_rect_x2=1085, u_rect_y2=425;
    int m_rect_x1=885, m_rect_y1=425, m_rect_x2=1085, m_rect_y2=625;
    ss<<"$"<<price_counter;
    //SDL_FillRect(screen , &workRect , SDL_MapRGB(screen->format , 200 , 200 , 200 ) );
    //SDL_FillRect(screen , &workRect2 , SDL_MapRGB(screen->format , 34 , 17 , 34 ) );
    //SDL_FillRect(screen , &hRect , SDL_MapRGB(screen->format , 34 , 100 , 34 ) );
    //gfxPrimitivesSetFont(&SDL_gfx_font_5x7_fnt,5,7);
    stringColor(screen, 1200, 225, ss.str().c_str(), SDL_MapRGB(screen->format , 200 , 250 , 250 ));

    switch(state) {
    case 0:
        roundedRectangleColor(screen, m_rect_x1, m_rect_y1, m_rect_x2,
                              m_rect_y2, 20, SDL_MapRGB(screen->format , 200 , 250 , 250 ));
        rectangleColor(screen, u_rect_x1, u_rect_y1, u_rect_x2,
                       u_rect_y2, SDL_MapRGB(screen->format , 200 , 200 , 200 ));
        break;

    case 1:
        roundedRectangleColor(screen,u_rect_x1, u_rect_y1, u_rect_x2,
                              u_rect_y2, 20, SDL_MapRGB(screen->format , 200 , 250 , 250 ));
        rectangleColor(screen, m_rect_x1, m_rect_y1, m_rect_x2,
                       m_rect_y2, SDL_MapRGB(screen->format , 200 , 200 , 200 ));
        break;
    }

    SDL_FillRect(screen , redRect , SDL_MapRGB(screen->format , 200 , 20 , 200 ) );
    SDL_Flip(screen);


}

