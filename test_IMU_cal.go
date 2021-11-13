package main

import (
	"fmt"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/johnusher/priWand/pkg/bno055_2"
	log "github.com/sirupsen/logrus"
	// "github.com/kpeu3i/bno055"
)

type AxisConfig struct {
	X     byte
	Y     byte
	Z     byte
	SignX byte
	SignY byte
	SignZ byte
}

func main() {
	sensor, err := bno055_2.NewSensor(0x28, 3)
	if err != nil {
		panic(err)
	}

	err = sensor.UseExternalCrystal(true)
	if err != nil {
		panic(err)
	}

	var (
		isCalibrated       bool
		calibrationOffsets bno055_2.CalibrationOffsets
		calibrationStatus  *bno055_2.CalibrationStatus
	)

	signals := make(chan os.Signal, 1)
	signal.Notify(signals, syscall.SIGINT, syscall.SIGTERM)

	err = sensor.EsetOperationMode(0x0C)
	if err != nil {
		panic(err)
	}

	axisConfig, err := sensor.AxisConfig()
	if err != nil {
		panic(err)
	}

	log.Infof("axisConfig: %v", axisConfig)

	err = sensor.RemapAxis(axisConfig)
	if err != nil {
		panic(err)
	}
	// log.Infof("mm: %v", mm)

	// mm, err := axis_config.Mappings(0x04)
	// if err != nil {
	// 	panic(err)
	// }
	// log.Infof("mm: %v", mm)

	// mappings := 0x04
	// mappings |= (c.Z & 0x03) << 4
	// mappings |= (c.Y & 0x03) << 2
	// mappings |= c.X & 0x03
	// log.Infof("mappings: %v", mm)

	// mm, err := sensor.AxisConfig()
	// if err != nil {
	// 	panic(err)
	// }

	// log.Infof("mm2: %v", mm)

	fmt.Printf("*** Done! Calibration offsets: %v\n", calibrationOffsets)

	// err = sensor.RemapAxis(0x0C)
	// if err != nil {
	// 	panic(err)
	// }

	// err = sensor.bus.Write(bno055AxisMapConfig, config.Mappings())
	// if err != nil {
	// 	return err
	// }

	// // here we set the sign
	// err =sensor.bus.Write(bno055AxisMapSign, config.Signs())
	// if err != nil {
	// 	return err
	// }

	// fmt.Printf("*** OG Calibration offsets: %v\n", bno055_2.CalibrationOffsets)

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
	}

	fmt.Printf("*** Done! Calibration offsets: %v\n", calibrationOffsets)
	// 3 = fully calibrated

	// Output
	// *** Calibration status (0..3): system=3, accelerometer=3, gyroscope=3, magnetometer=3
	// *** Done! Calibration offsets: [...]
}
