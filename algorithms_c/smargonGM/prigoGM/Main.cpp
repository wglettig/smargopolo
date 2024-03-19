#include <new.h>
#include "main.h"
#include "resource.h"
#include <stdio.h>

HINSTANCE TheInstance = 0;

int NewHandler (size_t size)
{
    throw WinException ( "Out of memory" );
    return 0;
}

Controller::Controller (HWND hwnd)
    :	
        _shx		 (hwnd,	IDC_EDIT_SHX),
        _shy		 (hwnd,	IDC_EDIT_SHY),
        _shz		 (hwnd,	IDC_EDIT_SHZ),
        _chi		 (hwnd,	IDC_EDIT_CHI),
        _ox			 (hwnd,	IDC_EDIT_OX),
        _oy			 (hwnd,	IDC_EDIT_OY),
        _oz			 (hwnd,	IDC_EDIT_OZ),
		
		_button_calc (hwnd,	IDC_BUTTON_CALC),

		_q1			 (hwnd,	IDC_STATIC_Q1),
        _q2			 (hwnd,	IDC_STATIC_Q2),
        _q3			 (hwnd,	IDC_STATIC_Q3),
        _q4			 (hwnd,	IDC_STATIC_Q4),
        _phimotor	 (hwnd,	IDC_STATIC_PHI),
        _omegamotor	 (hwnd,	IDC_STATIC_OMEGA),
        
		_msgbox		 (hwnd,	IDC_MSGBOX),
	
		_button_law (hwnd,	IDC_BUTTON_LAW),
		_button_quit(hwnd,	IDC_BUTTON_QUIT)
{
    // Attach icon to main dialog
    HICON hIcon = LoadIcon (TheInstance, MAKEINTRESOURCE (DLG_ICON));
    SendMessage (hwnd, WM_SETICON, WPARAM (TRUE), LPARAM (hIcon));
    hIcon = LoadIcon (TheInstance, MAKEINTRESOURCE (DLG_ICON_S));
    SendMessage (hwnd, WM_SETICON, WPARAM (FALSE), LPARAM (hIcon));

    // Other initializations...
	char tmp [1000];
	
	if (MyPrigo.readParamFile("prigoparameters.txt")!=0){
		sprintf(tmp,"Could not open 'prigoparameters.txt'. Using Default Values.");
		_msgbox.SetString(tmp);
	} else {
		sprintf(tmp,"'prigoparameters.txt' loaded.Ready...");
		_msgbox.SetString(tmp);
	}
	/*sprintf(tmp,"OffsetS:\r\n%.3lf\r\n%.3lf\r\n%.3lf\r\n%.3lf",	MyPrigo.getOffsetS1(),
															MyPrigo.getOffsetS2(),
															MyPrigo.getOffsetS3(),
															MyPrigo.getOffsetS4());
	*/

	//get initial Values from prigo model
	sprintf(tmp, "%.3lf", MyPrigo.Ox);
	_ox.SetString(tmp);
	sprintf(tmp, "%.3lf", MyPrigo.Oy);
	_oy.SetString(tmp);
	sprintf(tmp, "%.3lf", MyPrigo.Oz);
	_oz.SetString(tmp);
	sprintf(tmp, "%.3lf", MyPrigo.SHx);
	_shx.SetString(tmp);
	sprintf(tmp, "%.3lf", MyPrigo.SHy);
	_shy.SetString(tmp);
	sprintf(tmp, "%.3lf", MyPrigo.SHz);
	_shz.SetString(tmp);

	//CHI is not in param file
	_chi.SetString("0.000");

    _shx.SetFocus();
	_shx.Select();


}

void Controller::Command (HWND hwnd, int controlID, int command)
{
	//static prigo MyPrigo;
    
	switch (controlID)
    {        
		case IDC_BUTTON_CALC:
			if (command == BN_CLICKED)
			{
				//ACTION! CALCULATE!!!
				//Read in Fields
				char tmp[50];
				double UCS[10] = {0.,0.,0.,0.,0.,0.,0.};
				double MCS[7]  = {0.,0.,0.,0.,0.,0.,0.};

				_ox.GetString(tmp,20);
				sscanf(tmp,"%lf", &UCS[1]);
				sprintf(tmp,"%.3f", UCS[1]);
				_ox.SetString(tmp);

				_oy.GetString(tmp,20);
				sscanf(tmp,"%lf", &UCS[2]);
				sprintf(tmp,"%.3f", UCS[2]);
				_oy.SetString(tmp);

				_oz.GetString(tmp,20);
				sscanf(tmp,"%lf", &UCS[3]);
				sprintf(tmp,"%.3f", UCS[3]);
				_oz.SetString(tmp);

				_shx.GetString(tmp,20);
				sscanf(tmp,"%lf", &UCS[4]);
				sprintf(tmp,"%.3f", UCS[4]);
				_shx.SetString(tmp);

				_shy.GetString(tmp,20);
				sscanf(tmp,"%lf", &UCS[5]);
				sprintf(tmp,"%.3f", UCS[5]);
				_shy.SetString(tmp);

				_shz.GetString(tmp,20);
				sscanf(tmp,"%lf", &UCS[6]);
				sprintf(tmp,"%.3f", UCS[6]);
				_shz.SetString(tmp);

				_chi.GetString(tmp,20);
				sscanf(tmp,"%lf", &UCS[7]);
				sprintf(tmp,"%.3f", UCS[7]);
				_chi.SetString(tmp);

				//MyPrigo.
				int errMsg;
				char msg [200];
				errMsg = MyPrigo.mgi(UCS, MCS, msg);
				if (!errMsg) {
					//Print to Statics
					sprintf(tmp,"%.3lf", MCS[1]);
					_q1.SetString(tmp);
					sprintf(tmp,"%.3lf", MCS[2]);
					_q2.SetString(tmp);
					sprintf(tmp,"%.3lf", MCS[3]);
					_q3.SetString(tmp);
					sprintf(tmp,"%.3lf", MCS[4]);
					_q4.SetString(tmp);
					sprintf(tmp,"%.3lf", MCS[5]);
					_phimotor.SetString(tmp);
					sprintf(tmp,"%.3lf", MCS[6]);
					_omegamotor.SetString(tmp);
					
					_msgbox.SetString(msg);
	

				} else {
					sprintf(tmp,"Error occured (error no.):%d", errMsg);
					_msgbox.SetString(tmp);
				}



			}
		break;
		case IDC_BUTTON_LAW:
			if (command == BN_CLICKED)
			{
				//Make Prigo Law sweep file
				//open prigoLaw.txt File:
				if (MyPrigo.switchOn_CATIALawFile("prigoLaw.txt")==0){
					char tmp[100];
					double O[3];
					//get O values from input boxes
					_ox.GetString(tmp,20);	sscanf(tmp,"%lf", &O[0]);
					_oy.GetString(tmp,20);	sscanf(tmp,"%lf", &O[1]);
					_oz.GetString(tmp,20);	sscanf(tmp,"%lf", &O[2]);

					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,20.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25.,  0.,0.,0.);

					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,25.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5., 5.,25.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5.,-5.,25.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5.,-5.,25.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5., 5.,25.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,25.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25.,  0.,0.,0.);

					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16.,  0.,0.,0.);

					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,16.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5., 5.,16.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5.,-5.,16.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5.,-5.,16.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5., 5.,16.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,16.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16.,  0.,0.,0.);

					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25.,  0.,0.,0.);
					//sweep
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25., 15.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25., 30.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25., 45.,0.,0.);
					

					//intermediate 45°
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,25.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5., 5.,25.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5.,-5.,25.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5.,-5.,25.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5., 5.,25.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,25.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25.,  45.,0.,0.);

					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16.,  45.,0.,0.);

					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,16.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5., 5.,16.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5.,-5.,16.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5.,-5.,16.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5., 5.,16.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,16.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16.,  45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25.,  45.,0.,0.);
					//Intermediate
					
					
					
					
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25., 60.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25., 75.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25., 90.,0.,0.);

					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,25.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5., 5.,25.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5.,-5.,25.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5.,-5.,25.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5., 5.,25.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,25.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,25.,  90.,0.,0.);

					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16.,  90.,0.,0.);

					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,16.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5., 5.,16.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2], -5.,-5.,16.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5.,-5.,16.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  5., 5.,16.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 5.,16.,  90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16.,  90.,0.,0.);

					//sweep back
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16., 90.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16., 75.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16., 60.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16., 45.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16., 30.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16., 15.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16.,  0.,0.,0.);

					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,16.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,20.,  0.,0.,0.);
					MyPrigo.CATIALawFile_writeLine(O[0],O[1],O[2],  0., 0.,20.,  0.,0.,0.);

					//Stop Writing And Close File 
					MyPrigo.switchOff_CATIALawFile();

					//Give Feedback in Message Box:
					sprintf(tmp,"prigoLaw.txt successfully written.");
					_msgbox.SetString(tmp);
				} else {
					char tmp[100];
					sprintf(tmp,"error: could not write prigoLaw.txt.");
					_msgbox.SetString(tmp);
				}
			}
		break;

		case IDC_BUTTON_QUIT:
			if (command == BN_CLICKED)
			{
				PostQuitMessage(0);	
			}
		break;
    }
}

BOOL CALLBACK DialogProc (HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    static Controller* control = 0;
    switch (message)
    {
    case WM_INITDIALOG:
        try
        {
            control = new Controller (hwnd);
        }
        catch (WinException e)
        {
            MessageBox (0, e.GetMessage (), "Exception", MB_ICONEXCLAMATION | MB_OK);
        }
        catch (...)
        {
            MessageBox (0, "Unknown", "Exception", MB_ICONEXCLAMATION | MB_OK);
			return -1;
        }
        return TRUE;
    case WM_COMMAND:
        control->Command(hwnd, LOWORD(wParam), HIWORD (wParam));
        return TRUE;
    case WM_DESTROY:
        PostQuitMessage(0);
        return TRUE;
    case WM_CLOSE:
        delete control;
        DestroyWindow (hwnd);
        return TRUE;
    }
    return FALSE;
}

int WINAPI WinMain
   (HINSTANCE hInst, HINSTANCE hPrevInst, char * cmdParam, int cmdShow)
{
    TheInstance = hInst;
    _set_new_handler (&NewHandler);

    HWND hDialog = 0;

    hDialog = CreateDialog (hInst, MAKEINTRESOURCE (DLG_MAIN), 0, (DLGPROC)DialogProc);
    if (!hDialog)
    {
        char buf [100];
        wsprintf (buf, "Error x%x", GetLastError ());
        MessageBox (0, buf, "CreateDialog", MB_ICONEXCLAMATION | MB_OK);
        return 1;
    }

    MSG  msg;
    int status;
    while ((status = GetMessage (&msg, 0, 0, 0)) != 0)
    {
        if (status == -1)
            return -1;
        if (!IsDialogMessage (hDialog, &msg))
        {
            TranslateMessage ( &msg );
            DispatchMessage ( &msg );
        }
    }

    return msg.wParam;
}