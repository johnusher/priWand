package acc

// IMU package: accelerometer, magnet etc sensor using the BNo055

import (
	"fmt"
	"os"
	"os/signal"
	"syscall"
	"time"

	// "github.com/kpeu3i/bno055"
	log "github.com/sirupsen/logrus"

	// "github.com/kpeu3i/bno055_2"
	"github.com/johnusher/priWand/pkg/bno055_2"
)

const (
	Pi = 3.14159265358979323846264338327950288419716939937510582097494459 // pi https://oeis.org/A000796
)

type ACC interface {
	Run() error
	Close() error
}

type ACCMessage struct {
	// Temp    int8
	Bearing float64
	Roll    float64
	Tilt    float64

	QuatW float64
	QuatX float64
	QuatY float64
	QuatZ float64
}

// type ACCMessage2 struct {
// 	// Temp    int8
// 	QuatW float64
// 	QuatX float64
// 	QuatY float64
// 	QuatZ float64
// }

type acc struct {
	acc chan<- ACCMessage
	// acc2   chan<- ACCMessage2
	Sensor *bno055_2.Sensor
}

func Init(accChan chan<- ACCMessage, mock bool) (ACC, error) {
	if mock {
		return initMockACC(accChan)
	}

	return initACC(accChan)
}

// func EInit(*bno055_2.Sensor) (ACC, error) {
// 	err := bno055_2.Einit()
// 	if err != nil {
// 		return nil, err
// 	}
// }

func initACC(accChan chan<- ACCMessage) (ACC, error) {

	sensor, err := bno055_2.NewSensor(0x28, 3)
	if err != nil {
		panic(err)
	}

	err = sensor.UseExternalCrystal(true)
	if err != nil {
		panic(err)
	}

	status, err := sensor.Status()
	if err != nil {
		panic(err)
	}

	fmt.Printf("*** ICU Status: system=%v, system_error=%v, self_test=%v\n", status.System, status.SystemError, status.SelfTest)

	// System Status (see section 4.3.58)
	// ---------------------------------
	// 0 = Idle
	// 1 = System Error
	// 2 = Initializing Peripherals
	// 3 = System Initialization
	// 4 = Executing Self-Test
	// 5 = Sensor fusion algorithm running
	// 6 = System running without fusion algorithms

	// System Error (see section 4.3.59)
	//---------------------------------
	// 0 = No error
	// 1 = Peripheral initialization error
	// 2 = System initialization error
	// 3 = Self test result failed
	// 4 = Register map value out of range
	// 5 = Register map address out of range
	// 6 = Register map write error
	// 7 = BNO low power mode not available for selected operation ion mode
	// 8 = Accelerometer power mode not available
	// 9 = Fusion algorithm configuration error
	// A = Sensor configuration error

	// Self Test Results
	// --------------------------------
	// 1 = test passed, 0 = test failed
	//
	// Bit 0 = Accelerometer self test
	// Bit 1 = Magnetometer self test
	// Bit 2 = Gyroscope self test
	// Bit 3 = MCU self test
	//
	// 15=0x0F = all good!

	// err = sensor.EsetOperationMode(0x08)
	err = sensor.EsetOperationMode(0x0C) // fast mag cal

	axisConfig, err := sensor.AxisConfig()
	if err != nil {
		panic(err)
	}

	// log.Infof("axisConfig: %v", axisConfig)

	// remap axis with X axis in poitning direction, this is placement P0
	err = sensor.RemapAxis(axisConfig)
	if err != nil {
		panic(err)
	}

	// enter loop to calibrate
	// time-out after 2 secs
	var (
		isCalibrated       bool
		calibrationOffsets bno055_2.CalibrationOffsets
		calibrationStatus  *bno055_2.CalibrationStatus
	)

	signals := make(chan os.Signal, 1)
	signal.Notify(signals, syscall.SIGINT, syscall.SIGTERM)

	calTimeNow := time.Now()
	calTimeNowNow := time.Now()

	for !isCalibrated {
		select {
		case <-signals:
			err := sensor.Close()
			if err != nil {
				panic(err)
			}
		default:
			calibrationOffsets, calibrationStatus, err = sensor.Calibration()
			if err != nil {
				panic(err)
			}

			isCalibrated = calibrationStatus.IsCalibrated()

			fmt.Printf(
				"\r*** isCalibrated=%v: Calibration status (0..3): system=%v, accelerometer=%v, gyroscope=%v, magnetometer=%v",
				isCalibrated,
				calibrationStatus.System,
				calibrationStatus.Accelerometer,
				calibrationStatus.Gyroscope,
				calibrationStatus.Magnetometer,
			)

			// fmt.Printf("Calibration offsets: %v\n", calibrationOffsets)
		}

		time.Sleep(100 * time.Millisecond)

		calTimeNowNow = time.Now()

		elapsedTime := calTimeNowNow.Sub(calTimeNow)

		if elapsedTime > 2000*time.Millisecond {
			isCalibrated = true
		}
	}

	fmt.Printf("*** Done! Calibration offsets: %v\n", calibrationOffsets)

	if err != nil {
		panic(err)
	}

	return &acc{
		accChan,
		// accChan2,
		sensor,
	}, nil

}

func (a *acc) Close() error {
	return a.Sensor.Close()
}

func (a *acc) Run() error {

	for {
		select {
		// case <-signals:
		// 	err := sensor.Close()
		// 	if err != nil {
		// 		panic(err)
		// 	}
		default:

			vector, err := a.Sensor.Euler()
			if err != nil {
				log.Errorf("acc error: %v", err)
			}

			bearing := float64(vector.X)
			roll := float64(vector.Y)
			tilt := float64(vector.Z)

			quat, err := a.Sensor.Quaternion()
			// https://github.com/adafruit/Adafruit_BNO055/blob/master/utility/quaternion.h
			if err != nil {
				log.Errorf("Quaternion error: %v", err)
			}

			// sw := strconv.FormatFloat(float64(quat.W), 'f', -1, 32)
			// sx = strconv.FormatFloat(float64(quat.X), 'f', -1, 32)
			// sy = strconv.FormatFloat(float64(quat.Y), 'f', -1, 32)
			// sz = strconv.FormatFloat(float64(quat.Z), 'f', -1, 32)

			// sw := strconv.FormatFloat(float64(quat.W), 'f', -1, 32)
			// sx = strconv.FormatFloat(float64(quat.X), 'f', -1, 32)
			// sy = strconv.FormatFloat(float64(quat.Y), 'f', -1, 32)
			// sz = strconv.FormatFloat(float64(quat.Z), 'f', -1, 32)

			// _, err = f3.WriteString(sw + " " + sx + " " + sy + " " + sz + "\n")

			quat_w := float64(quat.W)
			quat_x := float64(quat.X)
			quat_y := float64(quat.Y)
			quat_z := float64(quat.Z)
			// acc, err := a.Sensor.LinearAccelerometer()
			// if err != nil {
			// 	log.Errorf("acc error: %v", err)
			// }

			// gyro, err := sensor.Gyroscope()
			// if err != nil {
			// 	log.Errorf("acc error: %v", err)
			// }

			// if err != nil {
			// 	log.Errorf("acc error: %v", err)
			// }

			// temp, err := a.Sensor.Temperature()
			// if err != nil {
			// 	panic(err)
			// }

			a.acc <- ACCMessage{
				// Temp:    temp,
				Bearing: bearing,
				Roll:    roll,
				Tilt:    tilt,
				QuatW:   quat_w,
				QuatX:   quat_x,
				QuatY:   quat_y,
				QuatZ:   quat_z,
			}

			// a.acc2 <- ACCMessage2{
			// 	// Temp:    temp,
			// 	QuatW: quat_w,
			// 	QuatX: quat_x,
			// 	QuatY: quat_y,
			// 	QuatZ: quat_z,
			// }

		}

		time.Sleep(5 * time.Millisecond) // check this for TF model!
		// todo: how to have different time for Bearing vs Quats
	}

}
