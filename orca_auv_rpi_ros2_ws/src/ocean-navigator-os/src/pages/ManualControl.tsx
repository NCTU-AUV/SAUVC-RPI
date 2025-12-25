import { useState, useRef, useEffect, useCallback } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Slider } from "@/components/ui/slider";
import { Switch } from "@/components/ui/switch";
import { Label } from "@/components/ui/label";
import {
  Gamepad2,
  Camera,
  Video,
  Lightbulb,
  Radio as Sonar,
  ArrowUp,
  Power,
  Navigation,
} from "lucide-react";
import { toast } from "sonner";
import { useDemoMode } from "@/context/DemoModeContext";
import { useROSWebSocket } from "@/hooks/useROSWebSocket";

const Joystick = ({ isEnabled, color = "accent", onMove, title, labels, externalPosition = { x: 0, y: 0 } }) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);

  useEffect(() => {
    if (!isDragging) {
      setPosition(externalPosition);
    }
  }, [externalPosition, isDragging]);

  const handleMouseDown = () => {
    if (!isEnabled) return;
    setIsDragging(true);
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    if (!isDragging || !containerRef.current || !isEnabled) return;

    const rect = containerRef.current.getBoundingClientRect();
    const centerX = rect.width / 2;
    const centerY = rect.height / 2;

    const x = e.clientX - rect.left - centerX;
    const y = e.clientY - rect.top - centerY;

    const distance = Math.sqrt(x * x + y * y);
    const maxDistance = 80;

    let finalX = x;
    let finalY = y;

    if (distance > maxDistance) {
      const angle = Math.atan2(y, x);
      finalX = Math.cos(angle) * maxDistance;
      finalY = Math.sin(angle) * maxDistance;
    }

    setPosition({ x: finalX, y: finalY });
    onMove?.(finalX, finalY);
  };

  const handleMouseUp = () => {
    setIsDragging(false);
    setPosition({ x: 0, y: 0 });
    onMove?.(0, 0);
  };

  useEffect(() => {
    if (isDragging) {
      document.addEventListener("mousemove", handleMouseMove as any);
      document.addEventListener("mouseup", handleMouseUp);
      return () => {
        document.removeEventListener("mousemove", handleMouseMove as any);
        document.removeEventListener("mouseup", handleMouseUp);
      };
    }
  }, [isDragging, isEnabled]);

  return (
    <div className="space-y-3">
      <Label className="text-sm text-muted-foreground">{title}</Label>
      <div
        ref={containerRef}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        className={`aspect-square rounded-2xl border-2 ${isEnabled ? `border-${color} bg-${color}/5` : "border-border bg-muted/20"
          } flex items-center justify-center relative cursor-grab active:cursor-grabbing ${!isEnabled && "opacity-50 cursor-not-allowed"
          }`}
      >
        <div className={`w-20 h-20 rounded-full bg-${color}/20 border-2 border-${color} flex items-center justify-center`}>
          <div
            className={`w-8 h-8 rounded-full bg-${color} transition-transform duration-75 glow-cyan`}
            style={{
              transform: `translate(${position.x * 0.5}px, ${position.y * 0.5}px)`,
            }}
          />
        </div>
        <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
          <div className="absolute top-4 text-xs text-muted-foreground">{labels?.top || "W"}</div>
          <div className="absolute bottom-4 text-xs text-muted-foreground">{labels?.bottom || "S"}</div>
          <div className="absolute left-4 text-xs text-muted-foreground">{labels?.left || "A"}</div>
          <div className="absolute right-4 text-xs text-muted-foreground">{labels?.right || "D"}</div>
        </div>
      </div>
    </div>
  );
};

const ManualControl = () => {
  const { demoMode } = useDemoMode();

  // WebSocket Connection
  const { isConnected: isWsConnected, lastMessage, sendJsonMessage } = useROSWebSocket({
    url: "ws://" + window.location.hostname + ":80/websocket",
  });

  const [isManualMode, setIsManualMode] = useState(true);
  const [speed, setSpeed] = useState([50]);
  const [lightsOn, setLightsOn] = useState(false);
  const [recording, setRecording] = useState(false);
  const [leftJoystick, setLeftJoystick] = useState({ x: 0, y: 0 });
  const [rightJoystick, setRightJoystick] = useState({ x: 0, y: 0 });
  const keysPressed = useRef<Set<string>>(new Set());
  const [shortcutNotification, setShortcutNotification] = useState<string | null>(null);
  const shortcutTimeoutRef = useRef<NodeJS.Timeout>();

  // Live Data States
  const [cameraImage, setCameraImage] = useState<string | null>(null);
  const [transformData, setTransformData] = useState<{
    tx: number;
    ty: number;
    rotation: number;
    scale: number;
  } | null>(null);

  const isConnected = demoMode ? true : isWsConnected;

  // Handle WebSocket Messages
  useEffect(() => {
    if (lastMessage && lastMessage.type === "topic") {
      if (lastMessage.data.topic_name === "is_kill_switch_closed") {
        if (lastMessage.data.msg) {
          toast.error("Kill Switch Activated!");
          setIsManualMode(false);
        }
      }
      if (lastMessage.data.topic_name === "bottom_camera_image") {
        setCameraImage(`data:image/jpeg;base64,${lastMessage.data.msg}`);
      }
      if (lastMessage.data.topic_name === "total_transform_px") {
        const [tx, ty, rotation, scale] = lastMessage.data.msg;
        setTransformData({ tx, ty, rotation, scale });
      }
    }
  }, [lastMessage]);

  const sendWrenchCommand = useCallback((forceX: number, forceY: number, forceZ: number, torqueZ: number) => {
    if (!isWsConnected || !isManualMode) return;
    const factor = speed[0] / 100.0;
    sendJsonMessage({
      type: "topic",
      data: {
        topic_name: "set_output_wrench_at_center_N_Nm",
        msg: {
          force: { x: forceX * factor, y: forceY * factor, z: forceZ * factor },
          torque: { x: 0, y: 0, z: torqueZ * factor },
        },
      },
    });
  }, [isWsConnected, isManualMode, speed, sendJsonMessage]);

  useEffect(() => {
    if (!isManualMode) return;
    const interval = setInterval(() => {
      const forceX = -(leftJoystick.y / 80) * 50;
      const forceY = (leftJoystick.x / 80) * 50;
      const forceZ = (rightJoystick.y / 80) * 50;
      const torqueZ = (rightJoystick.x / 80) * 20;
      sendWrenchCommand(forceX, forceY, forceZ, torqueZ);
    }, 100);
    return () => clearInterval(interval);
  }, [leftJoystick, rightJoystick, isManualMode, sendWrenchCommand]);

  const shortcuts = {
    camera: { key: "C", action: "拍照", icon: "📷" },
    record: { key: "V", action: "錄影", icon: "🎥" },
    lights: { key: "L", action: "燈光", icon: "💡" },
    sonar: { key: "X", action: "聲納掃描", icon: "📡" },
    surface: { key: "Z", action: "返回水面", icon: "⬆️" },
  };

  const showShortcutNotification = (message: string) => {
    setShortcutNotification(message);
    if (shortcutTimeoutRef.current) clearTimeout(shortcutTimeoutRef.current);
    shortcutTimeoutRef.current = setTimeout(() => setShortcutNotification(null), 2000);
  };

  const handleModeSwitch = (checked: boolean) => {
    if (!isConnected) {
      toast.error("需要線纜連接才能切換到手動模式");
      return;
    }
    setIsManualMode(checked);
    toast.success(checked ? "已切換至手動控制模式" : "已切換至自動模式");
  };

  const handleEmergencyStop = () => {
    toast.warning("緊急停止已觸發！");
    setLeftJoystick({ x: 0, y: 0 });
    setRightJoystick({ x: 0, y: 0 });
    keysPressed.current.clear();
    sendWrenchCommand(0, 0, 0, 0);
  };

  const updateJoystickFromKeys = () => {
    const keys = keysPressed.current;
    let leftX = 0, leftY = 0, rightX = 0, rightY = 0;
    if (keys.has("a") || keys.has("A")) leftX -= 80;
    if (keys.has("d") || keys.has("D")) leftX += 80;
    if (keys.has("w") || keys.has("W")) leftY -= 80;
    if (keys.has("s") || keys.has("S")) leftY += 80;
    if (keys.has("q") || keys.has("Q")) rightX -= 80;
    if (keys.has("e") || keys.has("E")) rightX += 80;
    if (keys.has("r") || keys.has("R")) rightY -= 80;
    if (keys.has("f") || keys.has("F")) rightY += 80;

    setLeftJoystick({ x: leftX, y: leftY });
    setRightJoystick({ x: rightX, y: rightY });
  };

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (!isManualMode) {
        if (key === "c" || key === "v") {
          e.preventDefault();
          toast.info("手動模式已禁用");
        }
        return;
      }
      if (["w", "a", "s", "d", "q", "e", "r", "f"].includes(key)) {
        e.preventDefault();
        keysPressed.current.add(key);
        updateJoystickFromKeys();
      }
      if (key === "c") {
        e.preventDefault();
        showShortcutNotification(`${shortcuts.camera.icon} 快捷鍵: ${shortcuts.camera.action}`);
        toast.success("拍照已觸發");
      } else if (key === "v") {
        e.preventDefault();
        setRecording(!recording);
        showShortcutNotification(`${shortcuts.record.icon} 快捷鍵: ${recording ? "停止錄影" : "開始錄影"}`);
      } else if (key === "l") {
        e.preventDefault();
        setLightsOn(!lightsOn);
        showShortcutNotification(`${shortcuts.lights.icon} 快捷鍵: 燈光${!lightsOn ? "開啟" : "關閉"}`);
      } else if (e.code === "Space") {
        e.preventDefault();
        handleEmergencyStop();
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      if (!isManualMode) return;
      const key = e.key.toLowerCase();
      if (["w", "a", "s", "d", "q", "e", "r", "f"].includes(key)) {
        e.preventDefault();
        keysPressed.current.delete(key);
        updateJoystickFromKeys();
      }
    };

    document.addEventListener("keydown", handleKeyDown);
    document.addEventListener("keyup", handleKeyUp);
    return () => {
      document.removeEventListener("keydown", handleKeyDown);
      document.removeEventListener("keyup", handleKeyUp);
    };
  }, [isManualMode, speed, recording, lightsOn]);

  return (
    <div className="min-h-screen p-6 space-y-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="mb-8 flex items-center justify-between">
          <div>
            <h1 className="text-3xl font-bold">手動控制</h1>
            <p className="text-muted-foreground mt-1">遙控操作與實時數據監控</p>
          </div>
          {demoMode && (
            <Badge className="bg-accent/20 text-accent border border-accent/50">📺 展示模式</Badge>
          )}
        </div>

        {/* Status Line */}
        <Card className="glass p-6 mb-6">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-4">
              <div className="flex items-center gap-2">
                <Sonar className={`w-5 h-5 ${isConnected ? "text-success" : "text-destructive"}`} />
                <span className="font-semibold">系統連接狀態</span>
              </div>
              <Badge variant="outline" className={isConnected ? "border-success/50 text-success" : "border-destructive/50 text-destructive"}>
                {isConnected ? "已連接" : "未連接"}
              </Badge>
              {isConnected && (
                <div className="flex items-center gap-4 text-sm text-muted-foreground">
                  <span>WebSocket: Active</span>
                  <span>PWM Signal: Ready</span>
                </div>
              )}
            </div>
            <div className="flex items-center gap-3">
              <Label htmlFor="mode-switch">手動模式</Label>
              <Switch id="mode-switch" checked={isManualMode} onCheckedChange={handleModeSwitch} disabled={!isConnected} />
            </div>
          </div>
        </Card>

        {/* Live Data Grid */}
        <div className="grid lg:grid-cols-3 gap-6 mb-6">
          {/* Main Camera Feed */}
          <Card className="glass lg:col-span-2 p-6 space-y-4">
            <h3 className="text-lg font-semibold flex items-center gap-2">
              <Camera className="w-5 h-5 text-accent" />
              主攝影機實時畫面
            </h3>
            <div className="aspect-video rounded-lg bg-black/40 flex items-center justify-center border border-border/50 relative overflow-hidden">
              {cameraImage ? (
                <img src={cameraImage} alt="Live feed" className="w-full h-full object-contain" />
              ) : (
                <div className="text-center space-y-2">
                  <Camera className="w-12 h-12 text-muted-foreground/20 mx-auto animate-pulse" />
                  <p className="text-sm text-muted-foreground italic">等待串流數據...</p>
                </div>
              )}
              <div className="absolute top-4 left-4 flex flex-col gap-2">
                <Badge className="bg-destructive/90 animate-pulse">● LIVE</Badge>
                <Badge variant="outline" className="bg-black/40 backdrop-blur-sm border-white/20 text-white text-[10px]">
                  {transformData ? `X: ${transformData.tx.toFixed(0)} Y: ${transformData.ty.toFixed(0)}` : "Positioning..."}
                </Badge>
              </div>
            </div>
          </Card>

          {/* 3D Attitude Indicator Placeholder / Transform Data */}
          <Card className="glass lg:col-span-1 p-6 space-y-4">
            <h3 className="text-lg font-semibold flex items-center gap-2">
              <Navigation className="w-5 h-5 text-accent" />
              3D 姿態指示器
            </h3>
            <div className="aspect-square rounded-lg bg-muted/20 flex flex-col p-4 border border-border/50">
              <div className="flex-1 space-y-6">
                <div>
                  <p className="text-xs text-muted-foreground uppercase mb-2">Translation (PX)</p>
                  <div className="grid grid-cols-2 gap-4">
                    <div className="bg-black/20 p-2 rounded">
                      <p className="text-[10px] text-muted-foreground">X Axis</p>
                      <p className="text-xl font-mono font-bold text-accent">{transformData?.tx.toFixed(1) || "0.0"}</p>
                    </div>
                    <div className="bg-black/20 p-2 rounded">
                      <p className="text-[10px] text-muted-foreground">Y Axis</p>
                      <p className="text-xl font-mono font-bold text-accent">{transformData?.ty.toFixed(1) || "0.0"}</p>
                    </div>
                  </div>
                </div>
                <div>
                  <p className="text-xs text-muted-foreground uppercase mb-2">Rotation & Scale</p>
                  <div className="grid grid-cols-2 gap-4">
                    <div className="bg-black/20 p-2 rounded">
                      <p className="text-[10px] text-muted-foreground">Rotation</p>
                      <p className="text-xl font-mono font-bold text-primary">{transformData?.rotation.toFixed(2) || "0.00"}</p>
                    </div>
                    <div className="bg-black/20 p-2 rounded">
                      <p className="text-[10px] text-muted-foreground">Scale</p>
                      <p className="text-xl font-mono font-bold text-success">{transformData?.scale.toFixed(2) || "1.00"}</p>
                    </div>
                  </div>
                </div>
              </div>
              <div className="pt-4 border-t border-border/50 flex justify-between items-center text-[10px]">
                <span className="text-muted-foreground italic">Source: total_transform_node</span>
                <Badge variant="outline" className="h-4 text-[10px]">10Hz</Badge>
              </div>
            </div>
          </Card>
        </div>

        {/* Controls Grid */}
        <div className="grid lg:grid-cols-3 gap-6">
          <Card className="glass lg:col-span-2 p-6 space-y-6">
            <div className="flex items-center justify-between">
              <h3 className="text-lg font-semibold flex items-center gap-2">
                <Gamepad2 className="w-5 h-5 text-accent" />
                搖桿控制器
              </h3>
              <Badge variant="outline" className={isManualMode ? "border-accent/50 text-accent" : ""}>
                {isManualMode ? "已啟用" : "未啟用"}
              </Badge>
            </div>
            <div className="grid grid-cols-2 gap-6">
              <Joystick isEnabled={isManualMode} color="accent" onMove={(x, y) => setLeftJoystick({ x, y })} title="左搖桿 - 前後左右" labels={{ top: "W", bottom: "S", left: "A", right: "D" }} externalPosition={leftJoystick} />
              <Joystick isEnabled={isManualMode} color="primary" onMove={(x, y) => setRightJoystick({ x, y })} title="右搖桿 - 旋轉上下" labels={{ top: "R", bottom: "F", left: "Q", right: "E" }} externalPosition={rightJoystick} />
            </div>
            <div className="space-y-4">
              <div className="flex items-center justify-between">
                <Label>速度調節</Label>
                <Badge variant="outline">{speed[0]}%</Badge>
              </div>
              <Slider value={speed} onValueChange={setSpeed} min={0} max={100} step={10} disabled={!isManualMode} />
            </div>
            <Button onClick={handleEmergencyStop} className="w-full bg-destructive hover:bg-destructive/90 glow-orange" disabled={!isManualMode}>
              <Power className="mr-2 h-5 w-5" /> 緊急停止 (Space)
            </Button>
          </Card>

          <Card className="glass p-6 space-y-4">
            <h3 className="text-lg font-semibold">快捷功能</h3>
            <div className="space-y-3">
              <Button variant="outline" className="w-full justify-start" disabled={!isManualMode} onClick={() => toast.success("拍照已觸發")}>
                <Camera className="mr-2 h-4 w-4" /> 拍照 (C)
              </Button>
              <Button variant="outline" className={`w-full justify-start ${recording ? "border-destructive text-destructive" : ""}`} disabled={!isManualMode} onClick={() => setRecording(!recording)}>
                <Video className="mr-2 h-4 w-4" /> {recording ? "停止錄影" : "開始錄影"} (V)
              </Button>
              <div className="flex items-center justify-between p-3 rounded-lg border border-border">
                <div className="flex items-center gap-2">
                  <Lightbulb className="w-4 h-4" /> <span className="text-sm">燈光 (L)</span>
                </div>
                <Switch checked={lightsOn} onCheckedChange={setLightsOn} disabled={!isManualMode} />
              </div>
              <Button variant="outline" className="w-full justify-start border-primary/50 hover:bg-primary/10" disabled={!isManualMode} onClick={() => {
                sendJsonMessage({ type: 'action', data: { action_name: 'initialize_all_thrusters' } });
                toast.success("初始化推進器...");
              }}>
                <Power className="mr-2 h-4 w-4" /> 初始化推進器
              </Button>
            </div>
            <div className="pt-4 border-t border-border space-y-2 text-xs text-muted-foreground">
              <p className="font-semibold text-foreground">鍵盤快捷鍵:</p>
              <p>WASD / QE / RF - 移動控制</p>
              <p>C / V / L - 拍照/錄影/燈光</p>
              <p>Space - 緊急停止</p>
            </div>
          </Card>
        </div>

        {shortcutNotification && (
          <div className="fixed bottom-6 right-6 animate-in fade-in slide-in-from-bottom-2 duration-300">
            <Card className="glass p-4 bg-accent/90 border-accent/50 backdrop-blur-md">
              <div className="flex items-center gap-2 text-white font-semibold">
                <span className="text-lg">{shortcutNotification.split(" ")[0]}</span>
                <span>{shortcutNotification.substring(2)}</span>
              </div>
            </Card>
          </div>
        )}
      </div>
    </div>
  );
};

export default ManualControl;
