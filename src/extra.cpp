/*
#if DEBUG_CODE>0
        //Output Streams Files
        //virTarget  << initPosLeftEE[0] << "		" << initPosLeftEE[1] << "		" << initPosLeftEE[2] <<endl;
        //computeEEPos(); //Computing the End Effector Positions
        //curPosition  << curPosLeftEE[0] << "		" << curPosLeftEE[1] << "		" << curPosLeftEE[2] <<endl;
        //virTarget  << initPosLeftEE[0] << "		" << initPosLeftEE[1] << "		" << initPosLeftEE[2] <<endl;
#endif



#if DEBUG_CODE>0
        //Storing The Joing Angles to the Ouput Stream File
        for(int j = 0 ; j < jAnglesLT.size() ; j++){
                    jointsLoc << jAnglesLT.at(j) <<"		";}
                jointsLoc<<endl;
#endif


#if DEBUG_CODE>0
        //Output Streams Files
        //virTarget  << initPosRightEE[0] << "		" << initPosRightEE[1] << "		" << initPosRightEE[2] <<endl;
        //computeEEPos(); //Computing the End Effector Positions
        //curPosition  << curPosRightEE[0] << "		" << curPosRightEE[1] << "		" << curPosRightEE[2] <<endl;
        //virTarget  << initPosRightEE[0] << "		" << initPosRightEE[1] << "		" << initPosRightEE[2] <<endl;
#endif

#if DEBUG_CODE>0
        //Storing The Joing Angles to the Ouput Stream File
        for(int j = 0 ; j < jAnglesRT.size() ; j++){
                    jointsLoc << jAnglesRT.at(j) <<"		";}
                jointsLoc<<endl;
#endif


#if DEBUG_CODE>0
    //virTarget.close();
    //curPosition.close();
#endif



*/


//printf ("Adjusting Compliances 0 \n");
//KFORCE=0.0094;//0.06;//0.005 // 0.0094
//        ITERATION=4000;
//        RAMP_KONSTANT=0.0015;
//        t_dur=5;
//        KOMP_JANG=0.0009;
//        KOMP_WAISZT=0.0001;
//        //KOMP_WAISZT3=0.00002
//        KOMP_WAISZT2=0.0009;
//        J0H=800;//200; //400 //800
//J1H=0.52;
//        J2H=400; //1 //400;
//        J3H=20; //4.5 //1 //20;
//J4H=1; //4.5
//J5H=1;//200; //1
//J6H=0.41;//200; //0.041;
//        J7H=1;//400; //1
//        J8H=500; //1
//        J9H=500; //1


//        meanJanL[0] =  0.5200;
//            meanJanL[1] =  0.0000;

//            meanJanL[2] =  0.0000;
//            meanJanL[3] = -0.7854;
//            meanJanL[4] =  0.0000;
//            meanJanL[5] =  0.7098;

//            meanJanL[6] =  0.9730;
//            meanJanL[7] =  1.2000;
//            meanJanL[8] =  0.0000;
//            meanJanL[9] =  0.0000;



//            JHdL[0] = J0H;
//            JHdL[1] = 0.000041;
//            JHdL[2] = J2H;
//            JHdL[3] = 0.041;
//            JHdL[4] = 0.041;
//            JHdL[5] = 5;
//            JHdL[6] = 0.041;
//            JHdL[7] = 75;
//            JHdL[8] = J8H;
//            JHdL[9] = J9H;

//            Joint_FieldL[0]=(meanJanL[0]-JanL[0]) * JHdL[0]; // Multiply by Joint compliance

//            Joint_FieldL[1]=(meanJanL[1]-JanL[1]) * JHdL[1]; //0.52 / Modified in June at Crete
//            Joint_FieldL[2]=(meanJanL[2]-JanL[2]) * JHdL[2];  //1.8
//            Joint_FieldL[3]=(meanJanL[3]-JanL[3]) * JHdL[3];  //4.5 //0.95
//            Joint_FieldL[4]=(meanJanL[4]-JanL[4]) * JHdL[4];
//            Joint_FieldL[5]=(meanJanL[5]-JanL[5]) * JHdL[5]; // Multiply by Joint compliance
//            Joint_FieldL[6]=(meanJanL[6]-JanL[6]) * JHdL[6]; //0.52 / Modified in June at Crete
//            Joint_FieldL[7]=(meanJanL[7]-JanL[7]) * JHdL[7];  //1.8
//            Joint_FieldL[8]=(meanJanL[8]-JanL[8]) * JHdL[8];  //4.5 //0.95
//            Joint_FieldL[9]=(meanJanL[9]-JanL[9]) * JHdL[9];

//            Jvel[10]= 1*((ffLFK[0]*JacobL[0])+(ffLFK[1]*JacobL[10])+(ffLFK[2]*JacobL[20])+Joint_FieldL[0]);
//                Jvel[11]= 0*((ffLFK[0]*JacobL[1])+(ffLFK[1]*JacobL[11])+(ffLFK[2]*JacobL[21])+Joint_FieldL[1]);
//                Jvel[12]= 1*((ffLFK[0]*JacobL[2])+(ffLFK[1]*JacobL[12])+(ffLFK[2]*JacobL[22])+Joint_FieldL[2]);
//                Jvel[13]= KOMP_JANG*((ffLFK[0]*JacobL[3])+(ffLFK[1]*JacobL[13])+(ffLFK[2]*JacobL[23])+Joint_FieldL[3]);
//                Jvel[14]= KOMP_JANG*((ffLFK[0]*JacobL[4])+(ffLFK[1]*JacobL[14])+(ffLFK[2]*JacobL[24])+Joint_FieldL[4]);
//                Jvel[15]= KOMP_JANG*((ffLFK[0]*JacobL[5])+(ffLFK[1]*JacobL[15])+(ffLFK[2]*JacobL[25])+Joint_FieldL[5]);
//                Jvel[16]= KOMP_JANG*((ffLFK[0]*JacobL[6])+(ffLFK[1]*JacobL[16])+(ffLFK[2]*JacobL[26])+Joint_FieldL[6]);
//                Jvel[17]= KOMP_JANG*((ffLFK[0]*JacobL[7])+(ffLFK[1]*JacobL[17])+(ffLFK[2]*JacobL[27])+Joint_FieldL[7]);
//                Jvel[18]= KOMP_JANG*((ffLFK[0]*JacobL[8])+(ffLFK[1]*JacobL[18])+(ffLFK[2]*JacobL[28])+Joint_FieldL[8]);
//                Jvel[19]= KOMP_JANG*((ffLFK[0]*JacobL[9])+(ffLFK[1]*JacobL[19])+(ffLFK[2]*JacobL[29])+Joint_FieldL[9]);

