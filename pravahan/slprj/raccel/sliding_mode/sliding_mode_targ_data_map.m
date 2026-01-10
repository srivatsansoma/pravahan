    function targMap = targDataMap(),

    ;%***********************
    ;% Create Parameter Map *
    ;%***********************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc paramMap
        ;%
        paramMap.nSections           = nTotSects;
        paramMap.sectIdxOffset       = sectIdxOffset;
            paramMap.sections(nTotSects) = dumSection; %prealloc
        paramMap.nTotData            = -1;

        ;%
        ;% Auto data (rtP)
        ;%
            section.nData     = 9;
            section.data(9)  = dumData; %prealloc

                    ;% rtP.VaryingStateSpace_InitialCondition
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtP.SlidingModeControllerReachingLaw_Phi
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 1;

                    ;% rtP.SlidingModeControllerReachingLaw_ReachingRate
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 2;

                    ;% rtP.SlidingModeControllerReachingLaw_SlidingMatrix
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 3;

                    ;% rtP.Delay_InitialCondition
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 11;

                    ;% rtP.Constant2_Value
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 12;

                    ;% rtP.Constant4_Value
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 16;

                    ;% rtP.Constant5_Value
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 17;

                    ;% rtP.Constant6_Value
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 33;

            nTotData = nTotData + section.nData;
            paramMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (parameter)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        paramMap.nTotData = nTotData;



    ;%**************************
    ;% Create Block Output Map *
    ;%**************************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 1;
        sectIdxOffset = 0;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc sigMap
        ;%
        sigMap.nSections           = nTotSects;
        sigMap.sectIdxOffset       = sectIdxOffset;
            sigMap.sections(nTotSects) = dumSection; %prealloc
        sigMap.nTotData            = -1;

        ;%
        ;% Auto data (rtB)
        ;%
            section.nData     = 12;
            section.data(12)  = dumData; %prealloc

                    ;% rtB.cvsh2wppvp
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

                    ;% rtB.hxxlqejlqe
                    section.data(2).logicalSrcIdx = 1;
                    section.data(2).dtTransOffset = 4;

                    ;% rtB.frltof5w1d
                    section.data(3).logicalSrcIdx = 2;
                    section.data(3).dtTransOffset = 6;

                    ;% rtB.b5mch31ag1
                    section.data(4).logicalSrcIdx = 3;
                    section.data(4).dtTransOffset = 8;

                    ;% rtB.f05ndgvcfi
                    section.data(5).logicalSrcIdx = 4;
                    section.data(5).dtTransOffset = 12;

                    ;% rtB.lyqikrix1a
                    section.data(6).logicalSrcIdx = 5;
                    section.data(6).dtTransOffset = 16;

                    ;% rtB.mgthgo0xnm
                    section.data(7).logicalSrcIdx = 6;
                    section.data(7).dtTransOffset = 20;

                    ;% rtB.exm13eufw3
                    section.data(8).logicalSrcIdx = 7;
                    section.data(8).dtTransOffset = 24;

                    ;% rtB.cr1f4lq33r
                    section.data(9).logicalSrcIdx = 8;
                    section.data(9).dtTransOffset = 28;

                    ;% rtB.luz5pmgfey
                    section.data(10).logicalSrcIdx = 9;
                    section.data(10).dtTransOffset = 32;

                    ;% rtB.en3labafh0
                    section.data(11).logicalSrcIdx = 10;
                    section.data(11).dtTransOffset = 40;

                    ;% rtB.jbmxj2cv5i
                    section.data(12).logicalSrcIdx = 11;
                    section.data(12).dtTransOffset = 48;

            nTotData = nTotData + section.nData;
            sigMap.sections(1) = section;
            clear section


            ;%
            ;% Non-auto Data (signal)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        sigMap.nTotData = nTotData;



    ;%*******************
    ;% Create DWork Map *
    ;%*******************
    
        nTotData      = 0; %add to this count as we go
        nTotSects     = 4;
        sectIdxOffset = 1;

        ;%
        ;% Define dummy sections & preallocate arrays
        ;%
        dumSection.nData = -1;
        dumSection.data  = [];

        dumData.logicalSrcIdx = -1;
        dumData.dtTransOffset = -1;

        ;%
        ;% Init/prealloc dworkMap
        ;%
        dworkMap.nSections           = nTotSects;
        dworkMap.sectIdxOffset       = sectIdxOffset;
            dworkMap.sections(nTotSects) = dumSection; %prealloc
        dworkMap.nTotData            = -1;

        ;%
        ;% Auto data (rtDW)
        ;%
            section.nData     = 1;
            section.data(1)  = dumData; %prealloc

                    ;% rtDW.jizccaxfkn
                    section.data(1).logicalSrcIdx = 0;
                    section.data(1).dtTransOffset = 0;

            nTotData = nTotData + section.nData;
            dworkMap.sections(1) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.iogt3txuqe.LoggedData
                    section.data(1).logicalSrcIdx = 1;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.c1nf0gk2e3.LoggedData
                    section.data(2).logicalSrcIdx = 2;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(2) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.oslah5f3b3
                    section.data(1).logicalSrcIdx = 3;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.djuz2hac2i
                    section.data(2).logicalSrcIdx = 4;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(3) = section;
            clear section

            section.nData     = 2;
            section.data(2)  = dumData; %prealloc

                    ;% rtDW.hd5lfda2lr
                    section.data(1).logicalSrcIdx = 5;
                    section.data(1).dtTransOffset = 0;

                    ;% rtDW.dl0lhrufxm
                    section.data(2).logicalSrcIdx = 6;
                    section.data(2).dtTransOffset = 1;

            nTotData = nTotData + section.nData;
            dworkMap.sections(4) = section;
            clear section


            ;%
            ;% Non-auto Data (dwork)
            ;%


        ;%
        ;% Add final counts to struct.
        ;%
        dworkMap.nTotData = nTotData;



    ;%
    ;% Add individual maps to base struct.
    ;%

    targMap.paramMap  = paramMap;
    targMap.signalMap = sigMap;
    targMap.dworkMap  = dworkMap;

    ;%
    ;% Add checksums to base struct.
    ;%


    targMap.checksum0 = 2096759775;
    targMap.checksum1 = 2439401993;
    targMap.checksum2 = 1016586332;
    targMap.checksum3 = 2257312720;

