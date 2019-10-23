#include <iostream>
#include <string>

#include <ctre/Phoenix.h>
#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <DoubleSolenoid.h>
#include <Solenoid.h>
#include <math.h>

#include <WPILib.h>
#include <Spark.h>
#include <Victor.h>
#include <Talon.h>
#include <Timer.h>
#include "Constants.h"

using  namespace frc;

// Joystick assignments
#define THROTTLE_AXIS   1
#define TURN_AXIS       0
#define L_TRIGGER       2
#define R_TRIGGER       3

// Gamepad controller button assignments
#define A_button    1
#define B_button    2
#define X_button    3
#define Y_button    4
#define L_bumper    5
#define R_bumper    6
#define Back_button     7
#define Start_button    8
#define L_stick_button  9
#define R_stick_button  10

///Dpad on controller button assignments
#define upDpad          12
#define downDpad        13
#define leftDpad        14
#define rightDpad       15

const static double kToleranceDegrees = 0.50;

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * The SampleRobot class is the base of a robot application that will
 * automatically call your Autonomous and OperatorControl methods at the right
 * time as controlled by the switches on the driver station or the field
 * controls.
 */
class Robot : public frc::SampleRobot, public PIDOutput
{
	// Driver's Station
	Joystick *leftstick = new Joystick(0);
	Joystick *rightstick = new Joystick(1); //right stick for throttle
	Joystick *m_cogamepad = new Joystick(2);

	//elevator 
	WPI_TalonSRX *m_elevator = new WPI_TalonSRX(9);
	WPI_TalonSRX *m_bottomE = new WPI_TalonSRX(11);

	//camera
	cs::UsbCamera camera1;
	cs::UsbCamera camera2;
	cs::VideoSink server;

	bool prevTrigger = false;

  std::string _sb;
	int _loops = 0;

	bool _lastButton1 = false;
	bool _lastButton2 = false;
	bool _lastButton3 = false;
	bool value_False = false;
 
	bool XButton_toggle = false;
	bool AButton_toggle = false;
	bool BButton_toggle = false;
	bool YButton_toggle = false;
	bool R_bumper_toggle = false;

	bool collect = false;
	bool cargo_ship = false;
	bool cargo_front = false;
	bool low_goal = false;
	bool mid_goal = false; 
	bool high_goal = false;
	bool elv_Home = false;
	bool right_Stick = false;

	bool intake_State = false;
	bool intake_Toggle = false;

	bool loaded = false;
	bool BackClimb = false;
	bool BackClimb_Toggle = false;
	bool FrontClimb = false;
	bool FrontClimb_Toggle = false;

	/** save the target position to servo to */
	double targetPositionRotations;

	//intake
	WPI_TalonSRX *m_intake = new WPI_TalonSRX(10);

	// Drivetrain
	WPI_TalonSRX *m_leftFront = new WPI_TalonSRX(3);
	WPI_TalonSRX *m_leftmid = new WPI_TalonSRX(5);
	WPI_TalonSRX *m_leftRear = new WPI_TalonSRX(7);

	WPI_TalonSRX *m_rightFront = new WPI_TalonSRX(4);
	WPI_TalonSRX *m_rightmid = new WPI_TalonSRX(6);
	WPI_TalonSRX *m_rightRear = new WPI_TalonSRX(8);
	
	DifferentialDrive *m_robotDrive = new DifferentialDrive(*m_leftFront, *m_rightFront);

	// Pneumatics
	Solenoid *Back_Climb_Extend = new Solenoid (1,0);
	Solenoid *Tray_Extend = new Solenoid (1,1); 
	Solenoid *Shifter_High = new Solenoid (1,2);
	Solenoid *Talon_Open = new Solenoid (1,3);
	Solenoid *Front_Climb_Extend = new Solenoid (1,4);

	Solenoid *Back_Climb_Retract = new Solenoid (2,0);
	Solenoid *Tray_Retract =  new Solenoid (2,1);
	Solenoid *Shifter_Low = new Solenoid (2,2);
	Solenoid *Talon_Close = new Solenoid (2,3);
	Solenoid *Front_Climb_Retract = new Solenoid (2,4);

/* 
	DoubleSolenoid *Tray = new DoubleSolenoid(0, 1);
	DoubleSolenoid *Talons = new DoubleSolenoid(2, 3);
	DoubleSolenoid *Shifter = new DoubleSolenoid(4, 5);
	DoubleSolenoid *frontClimb = new DoubleSolenoid(6, 7);
	DoubleSolenoid *BackClimb = new DoubleSolenoid(7, 8);
 */

	//Digital Inputs
	DigitalInput stop_intake{9}; // bump switch
	DigitalInput home {1}; // limit switch

	// NAVX varialbes
	// NAVX
	PIDController *turnController;      // PID Controller
	double rotateToAngleRate;           // Current rotation rate

	// Variables used for Exponential Driving Algorithm
  float Throttle;
  float TurnRate;
  float Throttle_Curved;
  float TurnRate_Curved;
  float a; // Throttle curve constant
  float b; // TurnRate curve constant
  float c; // TurnRate limiting constant (%/100)

  // Flags to check conditions
  bool Talon_In;
  bool Talon_Toggle;
  bool retracted;
  bool tray_Toggle;
	bool elevator_Moving;
	bool elev_mode_toggle;
	bool mode_state; // true = ball mode, false = hatch mode
	bool intake_mode;

	//camera
/* 		private:
    static void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(640, 480);
        cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
        cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
        cv::Mat source;
        cv::Mat output;
        while(true) {
            cvSink.GrabFrame(source);
            cvtColor(source, output, cv::COLOR_BGR2GRAY);
            outputStreamStd.PutFrame(output);
        }
    }
*/

  // Assignment of different autonomous modes names
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";

	public:
	// Updating heading of NAVX
	virtual void PIDWrite(double output)
	{
		this->rotateToAngleRate = output;
	}

	Robot()
	{
		
/* 	void VisionThread()
   	{
      cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
      camera.SetResolution(640, 480);
      cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
      cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
      cv::Mat source;
      cv::Mat output;
      while(true) 
			{
        cvSink.GrabFrame(source);
        cvtColor(source, output, cv::COLOR_BGR2GRAY);
        outputStreamStd.PutFrame(output);
      }
    } 
*/
			// Note SmartDashboard is not initialized here, wait until
			// RobotInit to make SmartDashboard calls

		rotateToAngleRate = 0.0;           // Current rotation rate

		Throttle = 0.0;
		TurnRate = 0.0;
		Throttle_Curved = 0.0;
		TurnRate_Curved = 0.0;
		a = 0.0; // Throttle curve constant
		b = 0.0; // TurnRate curve constant
		c = 0.0; // TurnRate limiting constant (%/100)

		Talon_In = false;
		Talon_Toggle = false;
		retracted = false;
		tray_Toggle = false;
		elev_mode_toggle = false;
		mode_state = false; // defaults to hatch mode on initiation
		m_robotDrive->SetExpiration(0.1);

		m_leftRear->Follow(*m_leftFront);
		m_leftmid->Follow(*m_leftFront);
		m_rightRear->Follow(*m_rightFront);
		m_rightmid->Follow(*m_rightFront);
			
		m_bottomE->Follow(*m_elevator);

	// High gear position
		Shifter_High->Set(true);
		Shifter_Low->Set(false);

	// Holding hatch position
		Talon_Close->Set(true);
		Talon_Open->Set(false);

	// Level 2 arm retracted
		Back_Climb_Retract->Set(true);		//false
		Back_Climb_Extend->Set(false);		//true

	// Level 2 front retracted
		Front_Climb_Retract->Set(true);
		Front_Climb_Extend->Set(false);
        
		void RobotInit();
  	{			
		 	camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  		camera2 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
 	 		server = frc::CameraServer::GetInstance()->GetServer();
	 		camera1.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
 			camera2.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
  
			CameraServer::GetInstance()->StartAutomaticCapture();
			Talon_In = false;
			Talon_Toggle = false;
			retracted = false;
			tray_Toggle = false;
			elev_mode_toggle = false;
			mode_state = false; // defaults to hatch mode on initiation
			intake_mode = false;

			_lastButton1 = false;
			_lastButton2 = false;
			_lastButton3 = false;
			value_False = false;

			XButton_toggle = false;
			AButton_toggle = false;
			BButton_toggle = false;
			YButton_toggle = false;
			R_bumper_toggle = false;

			collect = false;
			cargo_ship = false;
			low_goal = false;
			mid_goal = false; 
			high_goal = false;
			elv_Home = false;
			right_Stick = false;

			intake_State = false;
			intake_Toggle = false;

			loaded = false;
			BackClimb = false;
			BackClimb_Toggle = false;
			FrontClimb = false;
			FrontClimb_Toggle = false;

/* 		std::thread visionThread(VisionThread);
      visionThread.detach();
*/

			m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault); 	//Default
			m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);		//My Auto
			frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


			/* lets grab the 360 degree position of the MagEncoder's absolute position */
			int absolutePosition = m_elevator->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		 	//use the low level API to set the quad encoder signal 
			m_elevator->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

		 	/*choose the sensor and sensor direction */
			m_elevator->ConfigSelectedFeedbackSensor(
			FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
			m_elevator->SetSensorPhase(false);

		 	/*set the peak and nominal outputs, 12V means full */
			m_elevator->ConfigNominalOutputForward(0, kTimeoutMs);
			m_elevator->ConfigNominalOutputReverse(0, kTimeoutMs);
			m_elevator->ConfigPeakOutputForward(1, kTimeoutMs);
			m_elevator->ConfigPeakOutputReverse(-1, kTimeoutMs);

			targetPositionRotations = 0;

			/* set closed loop gains in slot0 */  // Talon PIDs are sourced from Phoenix tuner
			//m_elevator->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
			//m_elevator->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
			//m_elevator->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
			//m_elevator->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
		
			
		}	// *****End of Void RobotInit()***** 

		/*
		 * This autonomous (along with the chooser code above) shows how to
		 * select between different autonomous modes using the dashboard. The
		 * sendable chooser code works with the Java SmartDashboard. If you
		 * prefer the LabVIEW Dashboard, remove all of the chooser code and
		 * uncomment the GetString line to get the auto name from the text box
		 * below the Gyro.
		 *
		 * You can add additional auto modes by adding additional comparisons to
		 * the if-else structure below with additional strings. If using the
		 * SendableChooser make sure to add them to the chooser code above as
		 * well.
		 */
		 
	}	// *****End of Robot Class*****

		void Autonomous() override
		{
			DriveCode();
		}
		void OperatorControl() override
		{
			DriveCode();
		}
	
	
//*********************************************************************************************************
//
//			Main robot code!!!
//
//*********************************************************************************************************	
	
		void DriveCode() 
		{
			m_robotDrive->SetSafetyEnabled(true);

			while(IsEnabled())
			{				
//************************************* CAMERA SWITCHING ********************************************************************		
				/* if (rightstick->GetRawButton(5) && !prevTrigger	)	
				{
   			printf("Setting camera 2\n");
    		server.SetSource(camera2);
  			} 
				else if (!rightstick->GetRawButton(5) && prevTrigger) 	
				{
   		 	printf("Setting camera 1\n");
    		server.SetSource(camera1);
  			}
  			prevTrigger = rightstick->GetRawButton(5); */

//************************************* EXPONENTIAL DRIVING ALGORITHM ********************************************************************
       	Throttle = leftstick->GetRawAxis(THROTTLE_AXIS);
       	TurnRate = rightstick->GetRawAxis(TURN_AXIS);
       	a = 0.20;   // (0.20)
       	b = 0.15;   // (0.18)
       	c = 0.87;   // (0.87) Limits turnRate to % of maximum theoretical TurnRate

       	// Throttle Curved: a*x^3 + (1-a)*x <- Algorithm used to smooth throttle and steering inputs
       	Throttle_Curved = ((a * pow(Throttle, 3) + (1-a) * Throttle));
       	// TurnRate Curved: Maximum turnRate limited by variable c
       	TurnRate_Curved = (b * pow(TurnRate, 3) + (1-b) * TurnRate) * c;
       	// Raw Throttle and TurnRate fed into algorithm, curved values used in ArcadeDrive module
       	m_robotDrive->ArcadeDrive(Throttle, -TurnRate_Curved);

				// The motors will be updated every 5ms
				frc::Wait(0.005);

//************************************* SHIFTING SEQUENCE ********************************************************************
				if (rightstick->GetRawButton(2))
				{
					// LOW gear
					Shifter_High->Set(false);
					Shifter_Low->Set(true);
					//Shifter->Set(DoubleSolenoid::kForward);
				}
				else
				{
					// HIGH gear
					Shifter_High->Set(true);
					Shifter_Low->Set(false);
					//Shifter->Set(DoubleSolenoid::kReverse);
				}

//******************************************************* Cargo Intake ************************************************************************
				//When we sense a ball, do the following.
				if (stop_intake.Get() == 0 && loaded == false) // sensor(stop_Intake) when pressed value returns 0
				{
					//Turn intake motor off
					m_intake->Set(0.0);
					loaded = true;

					//Set Elevator height
					mode_state = true; //ball mode
					
					if(intake_mode == true)
					{
						// Retract Tray
						Tray_Retract->Set(true);
						Tray_Extend->Set(false);
						tray_Toggle = false;
						retracted = true;

						collect = false;
						low_goal = true; //seeks low goal height
						mid_goal = false;
						high_goal = false;
						cargo_ship = false; 

						//Reset buttons
						XButton_toggle = false;
						AButton_toggle = false;
						BButton_toggle = false; 
						YButton_toggle = false;
						R_bumper_toggle = false;

						intake_mode = false;
				}

				}
	 			//When the driver wants to intake a ball
				 else if (rightstick->GetRawButton(4) && intake_State == false && loaded == false) // false = intake off , true = intake on
				{
				  //Turn on intake motor
					intake_Toggle = true;	
					m_intake->Set(0.7);
					
				 	// Extend Tray
					Tray_Retract->Set(false);		// 
					Tray_Extend->Set(true);
			    tray_Toggle = true;
					retracted = false;

					//Set Elevator height
					mode_state = true; //ball mode
					intake_mode = true;
					
					collect = true;  //collect height
					low_goal = false;
					mid_goal = false;
					high_goal = false;
					cargo_ship = false;
					cargo_front = false;

					//Reset buttons
					XButton_toggle = false;
					AButton_toggle = false;
					BButton_toggle = false; 
					YButton_toggle = false;
					R_bumper_toggle = false;

					// LOADING HATCH POSITTION Talons
					Talon_Close->Set(false);
					Talon_Open->Set(true);
				}
				else if (!rightstick->GetRawButton(4) && intake_State == false && intake_Toggle == true)
				{
					intake_State = true;
				}   
				else if (rightstick->GetRawButton(4) && intake_State == true)
				{
					intake_Toggle = false;
					m_intake->Set(0.0);
				}
				else if (!rightstick->GetRawButton(4) && intake_State == true && intake_Toggle == false)
				{
					intake_State = false;
				}

//************************************* Shoot ****************************************************************************************
			 	if (rightstick->GetRawButton(1))
				{
					m_intake->Set(1.0);
					loaded = false;
					intake_Toggle = false;
					intake_State = false;
				}
				else if (!rightstick->GetRawButton(1) && intake_Toggle == false)
				{
					m_intake->Set(0.0);
				} 

//*******************************************************Hatch Intake Sequence************************************************************************
				//When we sense a ball, do the following.
				if (leftstick->GetRawButton(5))
				{
				  //Turn off intake motor
					m_intake->Set(0.0);
					intake_Toggle = false;
					intake_State = false;

				 	// Extend Tray
					Tray_Retract->Set(false);		// 
					Tray_Extend->Set(true);
			    tray_Toggle = true;
					retracted = false;

					//Set Elevator height
					mode_state = false; //hatch mode
					
					collect = true;  //collect height
					low_goal = false;
					mid_goal = false;
					high_goal = false;
					cargo_ship = false;
					cargo_front = false;

					//Reset buttons
					XButton_toggle = false;
					AButton_toggle = false;
					BButton_toggle = false; 
					YButton_toggle = false;
					R_bumper_toggle = false;

					// LOADING HATCH POSITTION Talons
					Talon_Close->Set(false);
					Talon_Open->Set(true);
				}

//************************************* TALON CONTROL SEQUENCE ********************************************************
				if (leftstick->GetRawButton(2) && !leftstick->GetRawButton(3))
				{
					// LOADING HATCH POSITTION Talons
					Talon_Close->Set(false);
					Talon_Open->Set(true);
				}	
				if (leftstick->GetRawButton(3) && !leftstick->GetRawButton(2))
				{
					// HOLDING HATCH POSITION Talons
					Talon_Close->Set(true);
					Talon_Open->Set(false);
				}

//************************************* TOGGLING OF BACK CLIMBING SEQUENCE ********************************************************************
				if (m_cogamepad->GetRawButton(rightDpad) && !m_cogamepad->GetRawButton(leftDpad))
				{
					//LEVELL TWO RETRACT
					Back_Climb_Retract->Set(true);			//Defualt state
					Back_Climb_Extend->Set(false);
				}
				if (m_cogamepad->GetRawButton(leftDpad) && !m_cogamepad->GetRawButton(rightDpad))
				{
					//LEVEL TWO Deploy
					Back_Climb_Retract->Set(false);
					Back_Climb_Extend->Set(true);

				}
/* 				if (m_cogamepad->GetRawButton(L_bumper) && BackClimb == false) // false = open
				{
					// LEVEL 2 Deploy
					Back_Climb_Retract->Set(true); 			//default state
					Back_Climb_Extend->Set(false);
			    BackClimb_Toggle = true;
				}
				else if (!m_cogamepad->GetRawButton(L_bumper) && BackClimb == false && BackClimb_Toggle == true)
				{
					BackClimb = true;
				}
				else if (m_cogamepad->GetRawButton(L_bumper) && BackClimb == true)
				{
					// LEVEL 2 Retract
					Back_Climb_Retract->Set(false);
					Back_Climb_Extend->Set(true);
					BackClimb_Toggle = false;
				}
				else if (!m_cogamepad->GetRawButton(L_bumper) && BackClimb == true && BackClimb_Toggle == false)
				{
					BackClimb = false;
				} */
//************************************* TOGGLING OF FRONT CLIMBING SEQUENCE ********************************************************************
					if(m_cogamepad->GetRawButton(upDpad) && !m_cogamepad->GetRawButton(downDpad))
					{
						//LEVEL TWO RETRACT
						Front_Climb_Retract->Set(false);
						Front_Climb_Extend->Set(true);
					}
					if (m_cogamepad->GetRawButton(downDpad) && !m_cogamepad->GetRawButton(upDpad))
					{
						//LEVEL TWO DEPLOY
						Front_Climb_Retract->Set(true);
						Front_Climb_Extend->Set(false);
					}
	/* 			if (m_cogamepad->GetRawButton(L_stick_button) && FrontClimb == false) // false = open
				{
					// LEVEL 2 Deploy
					Front_Climb_Retract->Set(false);
					Front_Climb_Extend->Set(true);
			    FrontClimb_Toggle = true;
				}
				else if (!m_cogamepad->GetRawButton(L_stick_button) && FrontClimb == false && FrontClimb_Toggle == true)
				{
					FrontClimb = true;
				}
				else if (m_cogamepad->GetRawButton(L_stick_button) && FrontClimb == true)
				{
					// LEVEL 2 Retract
					Front_Climb_Retract->Set(true);
					Front_Climb_Extend->Set(false);
					FrontClimb_Toggle = false;
				}
				else if (!m_cogamepad->GetRawButton(L_stick_button) && FrontClimb == true && FrontClimb_Toggle == false)
				{
					FrontClimb = false;
				}
 */
//************************************* TOGGLING OF TRAY SEQUENCE ********************************************************************
				if (rightstick->GetRawButton(3) && retracted == false) // false = open
				{
					// EXTEND Tray
					Tray_Retract->Set(false);		// 
					Tray_Extend->Set(true);
			    tray_Toggle = true;
				}
				else if (!rightstick->GetRawButton(3) && retracted == false && tray_Toggle == true)
				{
					retracted = true;
				}
				else if (rightstick->GetRawButton(3) && retracted == true)
				{
					// RETRACT Tray
					Tray_Retract->Set(true);
			  	Tray_Extend->Set(false);
					tray_Toggle = false;
				}
				else if (!rightstick->GetRawButton(3) && retracted == true && tray_Toggle == false)
				{
					retracted = false;
				}

 //***********************************ELEVATOR************************
				double motorOutput = m_elevator->GetMotorOutputPercent();

				/* prepare line to print */
				_sb.append("\tout:");
				_sb.append(std::to_string(motorOutput));
				_sb.append("\tpos:");
				_sb.append(std::to_string(m_elevator->GetSelectedSensorPosition(kPIDLoopIdx)));

				/* if Talon is in position closed-loop, print some more info */
				if (m_elevator->GetControlMode() == ControlMode::Position)
				{
					/* append more signals to print when in speed mode. */
					_sb.append("\terrNative:");
					_sb.append(std::to_string(m_elevator->GetClosedLoopError(kPIDLoopIdx)));
					_sb.append("\ttrg:");
					_sb.append(std::to_string(targetPositionRotations));
				}
				/* print every ten loops, printing too much too fast is generally bad for performance */
				if (++_loops >= 10)
				{
					_loops = 0;
					printf("%s\n", _sb.c_str());
				}
				_sb.clear();
				_loops = 0;
							
//*********************************************Elevator Mode Toggle*****************************************************
				if (m_cogamepad->GetRawButton(Back_button))
				{
						mode_state = true;  //ball mode
				}
				if (m_cogamepad->GetRawButton(Start_button))
				{
					mode_state = false; //hatch mode
				}

//******************************************Toggle heights*******************************************************
				if (m_cogamepad->GetRawButton(X_button) && collect == false)
				{
					XButton_toggle = true;
					AButton_toggle = false;
					BButton_toggle = false; 
					YButton_toggle = false;
					R_bumper_toggle = false;
					right_Stick = false;
				}
				else if (!m_cogamepad->GetRawButton(X_button) && collect == false && XButton_toggle == true)
				{
					collect = true;
					low_goal = false;
					mid_goal = false;
					high_goal = false;
					cargo_ship = false;
					cargo_front = false;
				}
				//if (m_cogamepad->GetRawButton(A_button) && low_goal == false)
				if (m_cogamepad->GetRawButton(A_button) && cargo_front == false)
				{
					XButton_toggle = false;
					AButton_toggle = true;
					BButton_toggle = false; 
					YButton_toggle = false;
					R_bumper_toggle = false;
					right_Stick = false;
				}
				else if (!m_cogamepad->GetRawButton(A_button) && cargo_front == false && AButton_toggle == true && mode_state == false)
				{
					collect = false;
					low_goal = false;
					mid_goal = false;
					high_goal = false;
					cargo_ship = false;
					cargo_front = true;
				}
				else if (!m_cogamepad->GetRawButton(A_button) && low_goal == false && AButton_toggle == true && mode_state == true)
				{
					collect = false;
					low_goal = true;
					mid_goal = false;
					high_goal = false;
					cargo_ship = false;
					cargo_front = false;
				}
				if (m_cogamepad->GetRawButton(B_button) && mid_goal == false)
				{
					XButton_toggle = false;
					AButton_toggle = false;
					BButton_toggle = true; 
					YButton_toggle = false;
					R_bumper_toggle = false;
					right_Stick = false;
				}
				else if (!m_cogamepad->GetRawButton(B_button) && mid_goal == false && BButton_toggle == true)
				{
					collect = false;
					low_goal = false;
					mid_goal = true;
					high_goal = false;
					cargo_ship = false;
					cargo_front = false;
				}
				if (m_cogamepad->GetRawButton(Y_button) && high_goal == false)
				{
					XButton_toggle = false;
					AButton_toggle = false;
					BButton_toggle = false; 
					YButton_toggle = true;
					R_bumper_toggle = false;
					right_Stick = false;
				}
				else if (!m_cogamepad->GetRawButton(Y_button) && high_goal == false && YButton_toggle == true)
				{
					collect = false;
					low_goal = false;
					mid_goal = false;
					high_goal = true;
					cargo_ship = false;
					cargo_front = false;
				} 
				if (m_cogamepad->GetRawButton(R_bumper) && cargo_ship == false)
				{
					XButton_toggle = false;
					AButton_toggle = false;
					BButton_toggle = false; 
					YButton_toggle = false;
					R_bumper_toggle = true;
					right_Stick = false;
				}
				else if (!m_cogamepad->GetRawButton(R_bumper) && cargo_ship == false && R_bumper_toggle == true)
				{
					collect = false;
					low_goal = false;
					mid_goal = false;
					high_goal = false;
					cargo_ship = true;
					cargo_front = false;
				}

//********************************************elevator height manual control*********************************************
			//if (m_cogamepad->GetRawAxis(THROTTLE_AXIS) > -0.2 && m_cogamepad->GetRawAxis(THROTTLE_AXIS) < 0.2)
			//	{
			//		m_elevator->Set(0.0);

			//	}
			//else if (m_cogamepad->GetRawAxis(THROTTLE_AXIS) > 0.5) //positive value is joystick down
			//	{
			//		m_elevator->Set(-0.2);

			//	}
		 	//else if (m_cogamepad->GetRawAxis(THROTTLE_AXIS) < -0.5) //negative value is joystick up
			//{
			//	m_elevator->Set(0.2);
			//} 

//********************************************elevator height control*********************************************						
				if (collect == true && mode_state == true)
				{
					targetPositionRotations = 1500;
					m_elevator->Set(ControlMode::Position, targetPositionRotations);  
				}
				if(cargo_ship == true && mode_state == true)
				{
					targetPositionRotations = 11200; 
					m_elevator->Set(ControlMode::Position, targetPositionRotations);
				}
				if (low_goal == true && mode_state == true)
				{
					targetPositionRotations = 3000;
					m_elevator->Set(ControlMode::Position, targetPositionRotations);
				}
				if (mid_goal == true && mode_state == true)
				{
					targetPositionRotations = 20000;
					m_elevator->Set(ControlMode::Position, targetPositionRotations);
				}
				if (high_goal == true && mode_state == true) 
				{
					targetPositionRotations = 35000;
					m_elevator->Set(ControlMode::Position, targetPositionRotations);
				}

				//hatch height Settings 
				if (collect == true && mode_state == false)
				{
					targetPositionRotations = 4800;
					m_elevator->Set(ControlMode::Position, targetPositionRotations);
				}
				if(cargo_ship == true && mode_state == false)
				{
					targetPositionRotations = 4800;
					m_elevator->Set(ControlMode::Position, targetPositionRotations);
				}
				if(cargo_front == true && mode_state == false)
				{
					targetPositionRotations = 3400;
					m_elevator->Set(ControlMode::Position, targetPositionRotations);
				}
				if (low_goal == true && mode_state == false)
				{
					targetPositionRotations = 4800;
					m_elevator->Set(ControlMode::Position, targetPositionRotations);
				}
				if (mid_goal == true && mode_state == false)
				{
					targetPositionRotations = 21000;
					m_elevator->Set(ControlMode::Position, targetPositionRotations);
				}
				if (high_goal == true && mode_state == false)
				{
					targetPositionRotations = 35500;
					m_elevator->Set(ControlMode::Position, targetPositionRotations);
				}
			}	// ***** End of while (IsOperatorControl() && IsEnabled()) *****
		}	// ***** End of void OperatorControl() override *****
    

		/*
		 * Runs during test mode
		 */
		void Test() override {}

	private:

};

START_ROBOT_CLASS(Robot)
