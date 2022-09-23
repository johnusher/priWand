package acc

// IMU package: accelerometer, magnet etc sensor using the BNo055

import (
	"fmt"
	"time"

	// "github.com/kpeu3i/bno055"
	log "github.com/sirupsen/logrus"

	// "github.com/kpeu3i/bno055_2"
	//"github.com/johnusher/ardpifi/pkg/bno055_2"
	"github.com/johnusher/priWand/pkg/bno055_2"
)

const (
	Pi = 3.14159265358979323846264338327950288419716939937510582097494459 // pi https://oeis.org/A000796
)

type ACC interface {
	Run() error
	Close() error
	ResetAcc() error
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

	GravX float64
	GravY float64
	GravZ float64
}

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

func initACC(accChan chan<- ACCMessage) (ACC, error) {

	sensor, err := bno055_2.NewSensor(0x28, 3)
	if err != nil {
		panic(err)
	}

	err = sensor.UseExternalCrystal(true)
	if err != nil {
		panic(err)
	}

	// fmt.Printf("*** Statusx: system=%v, system_error=%v, self_test=%v\n", status.System, status.SystemError, status.SelfTest)

	axisConfig, err := sensor.AxisConfig()
	if err != nil {
		panic(err)
	}

	err = sensor.RemapAxis(axisConfig)

	fmt.Printf("axisConfig 1 \n")

	axisConfig, err = sensor.AxisConfig()
	if err != nil {
		panic(err)
	}

	status, err := sensor.Status()
	if err != nil {
		panic(err)
	}

	// mapConfig, err := sensor.bus.Read(0x41)
	// if err != nil {
	// 	return nil, err
	// }

	// signConfig, err := sensor.bus.Read(0x42)
	// if err != nil {
	// 	return nil, err
	// }

	// fmt.Printf("*** mapConfig=%v, signConfig=%v\n", mapConfig, signConfig)

	// err = sensor.EsetOperationMode(0x08)
	//err = sensor.EsetOperationMode(0x0C) // fast mag cal

	// err = sensor.EsetOperationMode(0x08)
	//err = sensor.EsetOperationMode(0x0C) // fast mag cal NDOF bno055OprMode

	//err = sensor.EsetOperationMode(0x05) // bno055OprMode is ACCGYRO = 0101 =0x5

	err = sensor.EsetOperationMode(0x08) // bno055OprMode is IMUPLUS = 1000 =0x8
	//err = sensor.EsetOperationMode(0x09) // bno055OprMode is compass = 1001 =0x9
	// err = sensor.EsetOperationMode(0x0A) // bno055OprMode is M4G = 1010 =0xA

	//xxxxxxxxxxx
	status, err = sensor.Status()
	if err != nil {
		panic(err)
	}
	fmt.Printf("*** Status IMU: system=%v, system_error=%v, self_test=%v\n", status.System, status.SystemError, status.SelfTest)
	//xxxxxxxxxxx

	if err != nil {
		panic(err)
	}

	return &acc{
		accChan,
		// accChan2,
		sensor,
	}, nil

}

func (a *acc) ResetAcc() error {
	log.Printf("ResetAcc")

	_, err := a.Sensor.AxisConfig()
	if err != nil {
		panic(err)
	}

	a.Sensor.EsetOperationMode(0x08)

	status, err := a.Sensor.Status()
	if err != nil {
		panic(err)
	}

	fmt.Printf("*** Statusx: system=%v, system_error=%v, self_test=%v\n", status.System, status.SystemError, status.SelfTest)

	return err
}

// err = sensor.EsetOperationMode(0x08) // bno055OprMode is IMUPLUS = 1000 =0x8

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

			grav, err := a.Sensor.Gravity()
			// https://github.com/adafruit/Adafruit_BNO055/blob/master/utility/quaternion.h
			if err != nil {
				log.Errorf("Gravity error: %v", err)
			}
			grav_x := float64(grav.X)
			grav_y := float64(grav.Y)
			grav_z := float64(grav.Z)

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
				GravX:   grav_x,
				GravY:   grav_y,
				GravZ:   grav_z,
			}

		}

		time.Sleep(5 * time.Millisecond) // check this for TF model!
	}

}
