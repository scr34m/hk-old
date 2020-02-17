package main

import (
	"github.com/brutella/hc"
	"github.com/brutella/hc/accessory"
	"github.com/brutella/hc/characteristic"
	"github.com/brutella/hc/log"
	"github.com/brutella/hc/service"
	"github.com/facchinm/go-serial"
)

type ServiceFan struct {
	Service       *service.Service
	On            *characteristic.On
	RotationSpeed *characteristic.RotationSpeed
}

func NewServiceFan() *ServiceFan {
	svc := ServiceFan{}
	svc.Service = service.New(service.TypeFan)

	svc.On = characteristic.NewOn()
	svc.RotationSpeed = characteristic.NewRotationSpeed()

	svc.Service.AddCharacteristic(svc.On.Characteristic)
	svc.Service.AddCharacteristic(svc.RotationSpeed.Characteristic)
	return &svc
}

type AccessoryFan struct {
	Accessory *accessory.Accessory
	Fan       *ServiceFan
}

func NewAccessoryFan(info accessory.Info) *AccessoryFan {
	acc := AccessoryFan{}
	acc.Accessory = accessory.New(info, accessory.TypeFan)

	acc.Fan = NewServiceFan()
	acc.Accessory.AddService(acc.Fan.Service)

	return &acc
}

func SetSpeed(port *serial.SerialPort, speed byte) {
	_, err := port.Write([]byte{speed})
	if err != nil {
		log.Info.Panic(err)
	}
}

func main() {

	mode := &serial.Mode{
		BaudRate: 9600,
	}
	port, err := serial.OpenPort("/dev/ttyUSB1", mode)
	if err != nil {
		log.Info.Panic(err)
	}

	info := accessory.Info{
		Name: "Server rack",
	}

	acc := NewAccessoryFan(info)
	acc.Fan.On.SetValue(true)
	acc.Fan.On.OnValueRemoteUpdate(func(on bool) {
		if on {
			SetSpeed(port, 70)
		} else {
			SetSpeed(port, 1)
		}
	})

	acc.Fan.RotationSpeed.SetValue(70)
	acc.Fan.RotationSpeed.OnValueRemoteUpdate(func(value float64) {
		SetSpeed(port, byte(value))
	})

	t, err := hc.NewIPTransport(hc.Config{Pin: "21111111"}, acc.Accessory)
	if err != nil {
		log.Info.Panic(err)
	}

	hc.OnTermination(func() {
		<-t.Stop()
	})

	t.Start()
}
