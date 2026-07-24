package arm

import "testing"

func TestValidate(t *testing.T) {
	tests := []struct {
		name    string
		cfg     Config
		wantErr bool
	}{
		{"missing host", Config{}, true},
		{"valid host", Config{Host: "1.2.3.4"}, false},
		{"speed too high", Config{Host: "h", SpeedFactor: 2}, true},
		{"speed negative", Config{Host: "h", SpeedFactor: -1}, true},
		{"speed valid", Config{Host: "h", SpeedFactor: 0.5}, false},
		{"unknown end effector", Config{Host: "h", EndEffector: "claw"}, true},
		{"known end effector", Config{Host: "h", EndEffector: "hand"}, false},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			_, _, err := tt.cfg.Validate("path")
			if (err != nil) != tt.wantErr {
				t.Fatalf("Validate() error = %v, wantErr %v", err, tt.wantErr)
			}
		})
	}
}

func TestValidateMotionDep(t *testing.T) {
	deps, opt, err := (&Config{Host: "h", Motion: "my-motion"}).Validate("path")
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if len(deps) != 1 || len(opt) != 0 {
		t.Fatalf("got deps %v, opt %v", deps, opt)
	}
}

func TestSpeedFactor(t *testing.T) {
	if got := (&Config{}).speedFactor(); got != defaultSpeed {
		t.Fatalf("got %v, want %v", got, defaultSpeed)
	}
	if got := (&Config{SpeedFactor: 0.5}).speedFactor(); got != 0.5 {
		t.Fatalf("got %v, want 0.5", got)
	}
}

func TestURDFPath(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", "")
	if got := (&Config{}).urdfPath(); got != "arm/"+defaultURDFFile {
		t.Fatalf("got %q", got)
	}
	if got := (&Config{URDFPath: "/custom.urdf"}).urdfPath(); got != "/custom.urdf" {
		t.Fatalf("got %q", got)
	}
	t.Setenv("VIAM_MODULE_ROOT", "/root")
	if got := (&Config{}).urdfPath(); got != "/root/arm/"+defaultURDFFile {
		t.Fatalf("got %q", got)
	}
}

func TestEndEffectorSTLPath(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", "")
	if got := endEffectorSTLPath("meshes/panda/hand.stl"); got != "arm/meshes/panda/hand.stl" {
		t.Fatalf("got %q", got)
	}
	t.Setenv("VIAM_MODULE_ROOT", "/root")
	if got := endEffectorSTLPath("meshes/panda/hand.stl"); got != "/root/arm/meshes/panda/hand.stl" {
		t.Fatalf("got %q", got)
	}
}
