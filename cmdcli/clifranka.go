// Package main is a small debug CLI for the Franka Panda module. Connects
// directly to the arm via the same Go wrapper the module uses, lets you
// exercise read/move/recover without spinning up viam-server.
//
// Examples:
//
//	clifranka -host 172.16.0.2 -read
//	clifranka -host 172.16.0.2 -recover
//	clifranka -host 172.16.0.2 -move-joint 6 -move-radians -0.1 -speed 0.05
package main

import (
	"context"
	"flag"
	"fmt"
	"os"

	frankaarm "github.com/viam-modules/viam-franka-arm/arm"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
)

func main() {
	if err := realMain(); err != nil {
		fmt.Fprintln(os.Stderr, "error:", err)
		os.Exit(1)
	}
}

func realMain() error {
	host := flag.String("host", "172.16.0.2", "FCI host IP")
	urdf := flag.String("urdf", "arm/panda_arm.urdf", "path to panda_arm.urdf")
	speed := flag.Float64("speed", 0.05, "libfranka speed_factor in (0, 1]")
	read := flag.Bool("read", false, "read once and print joint positions")
	recoverErrs := flag.Bool("recover", false, "automaticErrorRecovery")
	moveJoint := flag.Int("move-joint", -1, "joint index 0..6 to move (-1 disables)")
	moveRadians := flag.Float64("move-radians", 0.0, "delta in radians for -move-joint")
	flag.Parse()

	ctx := context.Background()
	logger := logging.NewLogger("clifranka")

	cfg := &frankaarm.Config{
		Host:        *host,
		SpeedFactor: *speed,
		URDFPath:    *urdf,
	}

	registration, ok := resource.LookupRegistration(arm.API, frankaarm.PandaModel)
	if !ok {
		return fmt.Errorf("panda model not registered")
	}
	conf := resource.Config{
		Name:                "panda-cli",
		API:                 arm.API,
		Model:               frankaarm.PandaModel,
		ConvertedAttributes: cfg,
	}
	armRes, err := registration.Constructor(ctx, nil, conf, logger)
	if err != nil {
		return fmt.Errorf("constructing panda: %w", err)
	}
	defer func() {
		_ = armRes.Close(ctx)
	}()

	armComp, ok := armRes.(arm.Arm)
	if !ok {
		return fmt.Errorf("constructed resource is not arm.Arm")
	}

	if *recoverErrs {
		if _, err := armComp.DoCommand(ctx, map[string]any{"recover": true}); err != nil {
			return fmt.Errorf("recover: %w", err)
		}
		fmt.Println("recovered")
	}

	pos, err := armComp.JointPositions(ctx, nil)
	if err != nil {
		return fmt.Errorf("read joints: %w", err)
	}
	fmt.Printf("joints (rad): %v\n", []referenceframe.Input(pos))

	if *read {
		return nil
	}

	if *moveJoint >= 0 && *moveJoint < 7 {
		target := append([]referenceframe.Input(nil), pos...)
		target[*moveJoint] = target[*moveJoint] + *moveRadians
		fmt.Printf("moving joint %d by %+.4f rad\n", *moveJoint, *moveRadians)
		if err := armComp.MoveToJointPositions(ctx, target, nil); err != nil {
			return fmt.Errorf("move: %w", err)
		}
		fmt.Println("done")
	}

	return nil
}
