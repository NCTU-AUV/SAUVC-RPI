import { useState, useRef, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Slider } from "@/components/ui/slider";
import { Switch } from "@/components/ui/switch";
import { Label } from "@/components/ui/label";
import {
  Radio,
  Gamepad2,
  Camera,
  Video,
  Lightbulb,
  Radio as Sonar,
  ArrowUp,
  Power,
} from "lucide-react";
import { toast } from "sonner";
import { useDemoMode } from "@/context/DemoModeContext";
import { demoManualControlData } from "@/data/demoData";

const Joystick = ({ isEnabled, color = "accent", onMove, title, labels, externalPosition = { x: 0, y: 0 } }) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);

  // 當外部位置改變時（鍵盤控制），更新搖桿位置
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

    // Constrain to circle
    const distance = Math.sqrt(x * x + y * y);
    const maxDistance = 80; // max radius of joystick

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
        className={`aspect-square rounded-2xl border-2 ${
          isEnabled ? `border-${color} bg-${color}/5` : "border-border bg-muted/20"
        } flex items-center justify-center relative cursor-grab active:cursor-grabbing ${
          !isEnabled && "opacity-50 cursor-not-allowed"
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
  const [isConnected] = useState(demoMode);
  const [isManualMode, setIsManualMode] = useState(false);
  const [speed, setSpeed] = useState([50]);
  const [lightsOn, setLightsOn] = useState(false);
  const [recording, setRecording] = useState(false);
  const [leftJoystick, setLeftJoystick] = useState({ x: 0, y: 0 });
  const [rightJoystick, setRightJoystick] = useState({ x: 0, y: 0 });
  const keysPressed = useRef<Set<string>>(new Set());
  const [shortcutNotification, setShortcutNotification] = useState<string | null>(null);
  const shortcutTimeoutRef = useRef<NodeJS.Timeout>();

  if (!demoMode) {
    return (
      <div className="min-h-screen p-6 space-y-6 flex items-center justify-center">
        <Card className="glass p-12 text-center max-w-md">
          <h1 className="text-2xl font-bold mb-4">手動控制</h1>
          <p className="text-muted-foreground mb-6">
            展示模式已關閉。請在系統設置中啟用展示模式或連接真實設備。
          </p>
          <p className="text-sm text-muted-foreground">
            真實 API 連接功能關閉中...
          </p>
        </Card>
      </div>
    );
  }

  // 快捷鍵配置
  const shortcuts = {
    camera: { key: "C", action: "拍照", icon: "📷" },
    record: { key: "V", action: "錄影", icon: "🎥" },
    lights: { key: "L", action: "燈光", icon: "💡" },
    sonar: { key: "X", action: "聲納掃描", icon: "📡" },
    surface: { key: "Z", action: "返回水面", icon: "⬆️" },
  };

  const showShortcutNotification = (message: string) => {
    setShortcutNotification(message);
    if (shortcutTimeoutRef.current) {
      clearTimeout(shortcutTimeoutRef.current);
    }
    shortcutTimeoutRef.current = setTimeout(() => {
      setShortcutNotification(null);
    }, 2000);
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
  };

  const handleLeftJoystickMove = (x: number, y: number) => {
    setLeftJoystick({ x, y });
    if (x !== 0 || y !== 0) {
      console.log(`左搖桿移動: X=${x.toFixed(2)}, Y=${y.toFixed(2)}, 速度=${speed[0]}%`);
    }
  };

  const handleRightJoystickMove = (x: number, y: number) => {
    setRightJoystick({ x, y });
    if (x !== 0 || y !== 0) {
      console.log(`右搖桿移動: X=${x.toFixed(2)}, Y=${y.toFixed(2)}`);
    }
  };

  // 鍵盤控制邏輯
  const updateJoystickFromKeys = () => {
    const keys = keysPressed.current;

    // 左搖桿 (WASD)
    let leftX = 0;
    let leftY = 0;

    if (keys.has("a") || keys.has("A")) leftX -= 80;
    if (keys.has("d") || keys.has("D")) leftX += 80;
    if (keys.has("w") || keys.has("W")) leftY -= 80;
    if (keys.has("s") || keys.has("S")) leftY += 80;

    // 規範化對角線移動
    if (leftX !== 0 && leftY !== 0) {
      const magnitude = Math.sqrt(leftX * leftX + leftY * leftY);
      leftX = (leftX / magnitude) * 80;
      leftY = (leftY / magnitude) * 80;
    }

    // 右搖桿 (QE-旋轉, RF-上下)
    let rightX = 0;
    let rightY = 0;

    if (keys.has("q") || keys.has("Q")) rightX -= 80;
    if (keys.has("e") || keys.has("E")) rightX += 80;
    if (keys.has("r") || keys.has("R")) rightY -= 80;
    if (keys.has("f") || keys.has("F")) rightY += 80;

    // 規範化對角線移動
    if (rightX !== 0 && rightY !== 0) {
      const magnitude = Math.sqrt(rightX * rightX + rightY * rightY);
      rightX = (rightX / magnitude) * 80;
      rightY = (rightY / magnitude) * 80;
    }

    setLeftJoystick({ x: leftX, y: leftY });
    setRightJoystick({ x: rightX, y: rightY });

    if (leftX !== 0 || leftY !== 0) {
      console.log(`[鍵盤] 左搖桿移動: X=${leftX.toFixed(0)}, Y=${leftY.toFixed(0)}, 速度=${speed[0]}%`);
    }
    if (rightX !== 0 || rightY !== 0) {
      console.log(`[鍵盤] 右搖桿移動: X=${rightX.toFixed(0)}, Y=${rightY.toFixed(0)}`);
    }
  };

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      const upperKey = e.key.toUpperCase();

      if (!isManualMode) {
        // 只允許快捷鍵在非手動模式下工作
        if (key === "c") {
          e.preventDefault();
          showShortcutNotification(`${shortcuts.camera.icon} 快捷鍵: ${shortcuts.camera.action}`);
          toast.info("手動模式已禁用");
        } else if (key === "v") {
          e.preventDefault();
          showShortcutNotification(`${shortcuts.record.icon} 快捷鍵: ${shortcuts.record.action}`);
          toast.info("手動模式已禁用");
        }
        return;
      }

      if (["w", "a", "s", "d", "q", "e", "r", "f"].includes(key)) {
        e.preventDefault();
        keysPressed.current.add(key);
        updateJoystickFromKeys();
      }

      // 快捷鍵控制
      if (key === "c") {
        e.preventDefault();
        showShortcutNotification(`${shortcuts.camera.icon} 快捷鍵: ${shortcuts.camera.action}`);
        toast.success("拍照已觸發");
      } else if (key === "v") {
        e.preventDefault();
        setRecording(!recording);
        showShortcutNotification(`${shortcuts.record.icon} 快捷鍵: ${recording ? "停止錄影" : "開始錄影"}`);
        toast.success(recording ? "錄影已停止" : "錄影已開始");
      } else if (key === "l") {
        e.preventDefault();
        setLightsOn(!lightsOn);
        showShortcutNotification(`${shortcuts.lights.icon} 快捷鍵: 燈光${!lightsOn ? "開啟" : "關閉"}`);
        toast.success(`燈光已${!lightsOn ? "開啟" : "關閉"}`);
      } else if (key === "x") {
        e.preventDefault();
        showShortcutNotification(`${shortcuts.sonar.icon} 快捷鍵: ${shortcuts.sonar.action}`);
        toast.success("聲納掃描已觸發");
      } else if (key === "z") {
        e.preventDefault();
        showShortcutNotification(`${shortcuts.surface.icon} 快捷鍵: ${shortcuts.surface.action}`);
        toast.success("返回水面已觸發");
      }

      // 緊急停止
      if (e.code === "Space") {
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
      if (shortcutTimeoutRef.current) {
        clearTimeout(shortcutTimeoutRef.current);
      }
    };
  }, [isManualMode, speed, recording, lightsOn]);

  return (
    <div className="min-h-screen p-6 space-y-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="mb-8">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-3xl font-bold">手動控制</h1>
              <p className="text-muted-foreground mt-1">遙控操作與連接管理</p>
            </div>
            {demoMode && (
              <Badge className="bg-accent/20 text-accent border border-accent/50">
                📺 展示模式
              </Badge>
            )}
          </div>
        </div>

        {/* Connection Status */}
        <Card className="glass p-6 mb-6">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-4">
              <div className="flex items-center gap-2">
                <Radio className={`w-5 h-5 ${isConnected ? "text-success" : "text-destructive"}`} />
                <span className="font-semibold">線纜連接狀態</span>
              </div>
              <Badge
                variant="outline"
                className={`${
                  isConnected
                    ? "border-success/50 text-success"
                    : "border-destructive/50 text-destructive"
                }`}
              >
                {isConnected ? "已連接" : "未連接"}
              </Badge>
              {isConnected && (
                <div className="flex items-center gap-4 text-sm text-muted-foreground">
                  <span>延遲: 45ms</span>
                  <span>帶寬: 良好</span>
                </div>
              )}
            </div>

            <div className="flex items-center gap-3">
              <Label htmlFor="mode-switch">手動模式</Label>
              <Switch
                id="mode-switch"
                checked={isManualMode}
                onCheckedChange={handleModeSwitch}
                disabled={!isConnected}
              />
            </div>
          </div>
        </Card>

        <div className="grid lg:grid-cols-3 gap-6">
          {/* Control Panel */}
          <Card className="glass lg:col-span-2 p-6 space-y-6">
            <div className="flex items-center justify-between">
              <h3 className="text-lg font-semibold flex items-center gap-2">
                <Gamepad2 className="w-5 h-5 text-accent" />
                虛擬搖桿控制器
              </h3>
              <Badge variant="outline" className={isManualMode ? "border-accent/50 text-accent" : ""}>
                {isManualMode ? "已啟用" : "未啟用"}
              </Badge>
            </div>

            {/* Joysticks */}
            <div className="grid grid-cols-2 gap-6">
              {/* Left Joystick */}
              <Joystick
                isEnabled={isManualMode}
                color="accent"
                onMove={handleLeftJoystickMove}
                title="左搖桿 - 前後左右"
                labels={{ top: "W", bottom: "S", left: "A", right: "D" }}
                externalPosition={leftJoystick}
              />

              {/* Right Joystick */}
              <Joystick
                isEnabled={isManualMode}
                color="primary"
                onMove={handleRightJoystickMove}
                title="右搖桿 - 旋轉上下"
                labels={{ top: "R", bottom: "F", left: "Q", right: "E" }}
                externalPosition={rightJoystick}
              />
            </div>

            {/* Speed Control */}
            <div className="space-y-4">
              <div className="flex items-center justify-between">
                <Label>速度調節</Label>
                <Badge variant="outline">{speed[0]}%</Badge>
              </div>
              <Slider
                value={speed}
                onValueChange={setSpeed}
                min={0}
                max={100}
                step={10}
                disabled={!isManualMode}
                className="w-full"
              />
              <div className="flex justify-between text-xs text-muted-foreground">
                <span>慢速</span>
                <span>中速</span>
                <span>快速</span>
              </div>
            </div>

            {/* Emergency Stop */}
            <Button
              onClick={handleEmergencyStop}
              className="w-full bg-destructive hover:bg-destructive/90 glow-orange"
              disabled={!isManualMode}
            >
              <Power className="mr-2 h-5 w-5" />
              緊急停止 (Space)
            </Button>
          </Card>

          {/* Quick Actions */}
          <Card className="glass p-6 space-y-4">
            <h3 className="text-lg font-semibold">快捷功能</h3>

            <div className="space-y-3">
              <Button
                variant="outline"
                className="w-full justify-start"
                disabled={!isManualMode}
                onClick={() => {
                  showShortcutNotification(`${shortcuts.camera.icon} 快捷鍵: ${shortcuts.camera.action}`);
                  toast.success("拍照已觸發");
                }}
              >
                <Camera className="mr-2 h-4 w-4" />
                拍照 (C)
              </Button>

              <Button
                variant="outline"
                className={`w-full justify-start ${recording ? "border-destructive text-destructive" : ""}`}
                disabled={!isManualMode}
                onClick={() => {
                  setRecording(!recording);
                  showShortcutNotification(`${shortcuts.record.icon} 快捷鍵: ${recording ? "停止錄影" : "開始錄影"}`);
                  toast.success(recording ? "錄影已停止" : "錄影已開始");
                }}
              >
                <Video className="mr-2 h-4 w-4" />
                {recording ? "停止錄影" : "開始錄影"} (V)
              </Button>

              <div className="flex items-center justify-between p-3 rounded-lg border border-border hover:bg-accent/5">
                <div className="flex items-center gap-2">
                  <Lightbulb className="w-4 h-4" />
                  <span className="text-sm">燈光 (L)</span>
                </div>
                <Switch
                  checked={lightsOn}
                  onCheckedChange={(checked) => {
                    setLightsOn(checked);
                    showShortcutNotification(`${shortcuts.lights.icon} 快捷鍵: 燈光${checked ? "開啟" : "關閉"}`);
                    toast.success(`燈光已${checked ? "開啟" : "關閉"}`);
                  }}
                  disabled={!isManualMode}
                />
              </div>

              <Button
                variant="outline"
                className="w-full justify-start"
                disabled={!isManualMode}
                onClick={() => {
                  showShortcutNotification(`${shortcuts.sonar.icon} 快捷鍵: ${shortcuts.sonar.action}`);
                  toast.success("聲納掃描已觸發");
                }}
              >
                <Sonar className="mr-2 h-4 w-4" />
                聲納掃描 (X)
              </Button>

              <Button
                variant="outline"
                className="w-full justify-start border-success/50 hover:bg-success/10"
                disabled={!isManualMode}
                onClick={() => {
                  showShortcutNotification(`${shortcuts.surface.icon} 快捷鍵: ${shortcuts.surface.action}`);
                  toast.success("返回水面已觸發");
                }}
              >
                <ArrowUp className="mr-2 h-4 w-4" />
                返回水面 (Z)
              </Button>
            </div>

            {/* Keyboard Shortcuts */}
            <div className="pt-4 border-t border-border space-y-2">
              <Label className="text-sm">鍵盤快捷鍵</Label>
              <div className="space-y-1 text-xs text-muted-foreground">
                <p><span className="font-semibold">移動控制:</span></p>
                <p className="ml-2">WASD - 前後左右移動</p>
                <p className="ml-2">QE - 左右旋轉</p>
                <p className="ml-2">RF - 上浮下潛</p>
                <p className="mt-2"><span className="font-semibold">快捷功能:</span></p>
                <p className="ml-2">C - 拍照</p>
                <p className="ml-2">V - 錄影</p>
                <p className="ml-2">L - 燈光</p>
                <p className="ml-2">X - 聲納掃描</p>
                <p className="ml-2">Z - 返回水面</p>
                <p className="ml-2">Space - 緊急停止</p>
              </div>
            </div>
          </Card>
        </div>

        {/* Shortcut Notification */}
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
