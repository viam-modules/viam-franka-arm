// Package main implements the viam-franka-arm module.
package main

import (
	frankaarm "github.com/viam-modules/viam-franka-arm/arm"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: arm.API, Model: frankaarm.PandaModel},
		resource.APIModel{API: gripper.API, Model: frankaarm.GripperModel},
	)
}
