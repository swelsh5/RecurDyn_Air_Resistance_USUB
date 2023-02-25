#include "stdafx.h"

#include "DllFunc.h"

#include "stdlib.h"

#include "math.h"

 

NodalForceExt_API void __cdecl nodal_force_ext

           (int id, double time, double upar[], int npar, int ifbody, int nodarr[], int nonde, int jflag, int iflag, double result[])

{

           using namespace rd_syscall;

           // Parameter Information

           //   id      :  Nodal Force sequential identification. (Input)

           //   time    :  Simulation time of RD/Solver. (Input)

           //   upar    :  Parameters defined by user. (Input)

           //   npar    :  Number of user parameters. (Input)

           //   ifbody  :  FFLEX Body sequential ID. (Input)

           //   nodarr  :  Node sequential ID array of input node set. (Input)

           //   nonde   :  Number of node of node set. (Input)

           //   jflag   :  When RD/Solver evaluates a Jacobian, the flag is true. (Input)

           //   iflag   :  When RD/Solver initializes arrays, the flag is true. (Input)

           //   result  :  Returned nodal force vector. Acting point of the nodal force is each center of each node.

           //            Reference frame of each force vector must be the ground inertia marker. (Output, Size : nonde * 6)

 

           // User Statement

           int i, j, nodeid,errflg;

           double Area, Cd, rho;

           double vn[3], vn_mag, ucf, scale;

           WCHAR NodeName[256];

           char  strNodeName[256];

 

           if (iflag) {

                     for(i=0;i<nonde;i++){

                                get_fflex_nodestringname(ifbody,nodarr[i],(TCHAR*)NodeName,&errflg);

                                wcstombs(strNodeName,NodeName,256);  // Wide-byte character --> Multibyte character

                                printmsg("========================",(int)strlen("==================="));

                                printmsg(strNodeName,(int)strlen(strNodeName));

                                printmsg("========================",(int)strlen("==================="));

                     }

           }

 

           Cd = upar[0];

           rho = upar[1];

           scale = upar[2];

           rd_ucf(&ucf);

 

           for(i=0;i<nonde;i++){

                     // Set velocity vector of node

                     get_fflex_nodetvel(ifbody,nodarr[i],NULL,0,vn,&errflg);

                     vn_mag = sqrt(pow(vn[0],2)+pow(vn[1],2)+pow(vn[2],2));

 

      get_fflex_nodeid(ifbody,nodarr[i],&nodeid,&errflg);

 

      if (nodeid == 10000 || nodeid == 10001)

      {

         Area = 100;

      }

      else if (nodeid >= 20000 && nodeid < 30000)

      {

         Area = 200;

      }

      else

         Area = 400;

 

                     for(j=0;j<3;j++){

                                if (vn_mag == 0.0)

                                           result[i*6+j] = 0.0;

                                else

                                           result[i*6+j] = -1.0/2.0*Cd*rho*vn_mag*vn_mag*Area

                                           *(vn[j]/vn_mag) // Set negative direction of the nodal velocity

                                           /ucf            // Making force unit

                                           *scale;         //

 

                                result[i*6+j+3] = 0.0;

                     }

           }

}