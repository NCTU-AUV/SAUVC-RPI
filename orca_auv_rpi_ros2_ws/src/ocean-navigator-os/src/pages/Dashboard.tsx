import { Card } from "@/components/ui/card";
import { Progress } from "@/components/ui/progress";
import { Badge } from "@/components/ui/badge";
import { Battery, Thermometer, Gauge as GaugeIcon, Navigation, Camera, Radio } from "lucide-react";
import { useDemoMode } from "@/context/DemoModeContext";
import { demoDashboardData } from "@/data/demoData";
import { useState, useEffect, useRef } from "react";






const VideoPlayer = () => {
  const videoRef = useRef<HTMLVideoElement | null>(null);

  useEffect(() => {
    const v = videoRef.current;
    if (!v) return;

    const log = (...args: unknown[]) => console.log("[VideoPlayer]", ...args);

    const tryPlay = async () => {
      try {
        // ensure muted before attempting autoplay
        v.muted = true;
        v.volume = 0;
        const p = v.play();
        if (p && typeof p.then === "function") {
          await p;
        }
        log("play succeeded", { paused: v.paused, currentTime: v.currentTime, readyState: v.readyState });
      } catch (err) {
        log("play failed", err);
      }
    };

    const onCanPlay = () => {
      log("canplay event fired, attempting play...");
      tryPlay();
    };

    const onLoaded = () => log("loadedmetadata", { duration: v.duration, videoWidth: v.videoWidth, videoHeight: v.videoHeight });
    const onPlay = () => log("onplay event", { paused: v.paused });
    const onPause = () => log("onpause event", { paused: v.paused });
    const onError = (e: Event) => log("onerror event", e);

    v.addEventListener("canplay", onCanPlay);
    v.addEventListener("loadedmetadata", onLoaded);
    v.addEventListener("play", onPlay);
    v.addEventListener("pause", onPause);
    v.addEventListener("error", onError);

    // try immediately in case already ready
    tryPlay();

    // as a fallback, try again after a short delay
    const retry = setTimeout(() => tryPlay(), 500);

    return () => {
      clearTimeout(retry);
      v.removeEventListener("canplay", onCanPlay);
      v.removeEventListener("loadedmetadata", onLoaded);
      v.removeEventListener("play", onPlay);
      v.removeEventListener("pause", onPause);
      v.removeEventListener("error", onError);
    };
  }, []);

  return (
    <video
      ref={videoRef}
      src="/src/data/test3.mp4"
      className="w-full h-full object-cover"
      playsInline
      preload="auto"
      muted
      loop
    />
  );
};

const Dashboard = () => {
  const { demoMode } = useDemoMode();
  const [data, setData] = useState(demoMode ? demoDashboardData : null);
  const [thrustersRPM, setThrustersRPM] = useState<number[]>(() =>
    new Array(8).fill(0).map(() => 1200)
  );
  const [missionProgress, setMissionProgress] = useState({
    percent: 0,
    completed: 0,
    total: 8,
    found: 0,
    seconds: 0,
  });

  useEffect(() => {
    if (!demoMode) return;
    const interval = setInterval(() => {
      setMissionProgress((prev) => {
        const nextPercent = Math.min(100, prev.percent + Math.random() * 2.5);
        const nextCompleted = Math.min(prev.total, Math.floor((nextPercent / 100) * prev.total));
        const nextFound = Math.min(nextCompleted, prev.found + (Math.random() < 0.1 ? 1 : 0));
        const nextSeconds = prev.seconds + 1;
        return {
          percent: nextPercent,
          completed: nextCompleted,
          total: prev.total,
          found: nextFound,
          seconds: nextSeconds,
        };
      });
    }, 1000);
    return () => clearInterval(interval);
  }, [demoMode]);

  useEffect(() => {
    if (!demoMode) return;
    const interval = setInterval(() => {
      setThrustersRPM((prev) =>
        prev.map((r) => {
          // randomly vary +/- up to 100 RPM, keep within 0-2000
          const delta = Math.round((Math.random() - 0.5) * 100);
          const next = Math.max(0, Math.min(2000, r + delta));
          return next;
        })
      );
    }, 800);
    return () => clearInterval(interval);
  }, [demoMode]);

  useEffect(() => {
    if (!demoMode) return;
    const interval = setInterval(() => {
      setData((prev) => {
        if (!prev) return prev;
        return {
          ...prev,
          status: {
            ...prev.status,
            battery: Math.max(10, Math.min(99, prev.status.battery + (Math.random() - 0.5) * 2)),
            depth: +(prev.status.depth + (Math.random() - 0.5) * 0.5).toFixed(1),
            pressure: +(prev.status.pressure + (Math.random() - 0.5) * 0.05).toFixed(2),
            temperature: +(prev.status.temperature + (Math.random() - 0.5) * 0.2).toFixed(1),
            latency: Math.max(10, Math.min(120, prev.status.latency + Math.floor((Math.random() - 0.5) * 10))),
            bandwidth: prev.status.bandwidth,
            connection: prev.status.connection,
            batteryRemaining: prev.status.batteryRemaining,
          },
          attitude: {
            pitch: +(prev.attitude.pitch + (Math.random() - 0.5) * 0.5).toFixed(1),
            roll: +(prev.attitude.roll + (Math.random() - 0.5) * 0.5).toFixed(1),
            yaw: +(prev.attitude.yaw + (Math.random() - 0.5) * 2).toFixed(0),
          },
          videoStream: prev.videoStream,
        };
      });
    }, 1000);
    return () => clearInterval(interval);
  }, [demoMode]);

  if (!demoMode) {
    return (
      <div className="min-h-screen p-6 space-y-6 flex items-center justify-center">
        <Card className="glass p-12 text-center max-w-md">
          <h1 className="text-2xl font-bold mb-4">監控儀表板</h1>
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
  return (
    <div className="min-h-screen p-6 space-y-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="mb-8">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-3xl font-bold">監控儀表板</h1>
              <p className="text-muted-foreground mt-1">AUV 實時狀態監控</p>
            </div>
            {demoMode && (
              <Badge className="bg-accent/20 text-accent border border-accent/50">
                📺 展示模式
              </Badge>
            )}
          </div>
        </div>

        {/* Status Cards */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
          <Card className="glass p-4 space-y-3">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-2">
                <Battery className="w-5 h-5 text-success" />
                <span className="text-sm font-medium">電池電量</span>
              </div>
              <Badge variant="outline" className="border-success/50 text-success">
                {data!.status.battery.toFixed(2)}%
              </Badge>
            </div>
            <Progress value={data!.status.battery} className="h-2" />
            <p className="text-xs text-muted-foreground">預估剩餘時間: {data!.status.batteryRemaining}</p>
          </Card>

          <Card className="glass p-4 space-y-3">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-2">
                <GaugeIcon className="w-5 h-5 text-accent" />
                <span className="text-sm font-medium">深度</span>
              </div>
              <Badge variant="outline" className="border-accent/50 text-accent">
                {data!.status.depth}m
              </Badge>
            </div>
            <div className="text-2xl font-bold">{data!.status.depth} 米</div>
            <p className="text-xs text-muted-foreground">水壓: {data!.status.pressure} bar</p>
          </Card>

          <Card className="glass p-4 space-y-3">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-2">
                <Thermometer className="w-5 h-5 text-primary" />
                <span className="text-sm font-medium">水溫</span>
              </div>
              <Badge variant="outline" className="border-primary/50">
                {data!.status.temperature}°C
              </Badge>
            </div>
            <div className="text-2xl font-bold">{data!.status.temperature} °C</div>
            <p className="text-xs text-muted-foreground">正常範圍</p>
          </Card>

          <Card className="glass p-4 space-y-3">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-2">
                <Radio className="w-5 h-5 text-success" />
                <span className="text-sm font-medium">連接狀態</span>
              </div>
              <div className="w-3 h-3 rounded-full bg-success animate-pulse glow-cyan" />
            </div>
            <div className="text-sm font-semibold text-success">{data!.status.connection}</div>
            <p className="text-xs text-muted-foreground">延遲: {data!.status.latency}ms | 帶寬{data!.status.bandwidth}</p>
          </Card>
        </div>

        {/* Main Content Grid */}
        <div className="grid lg:grid-cols-3 gap-6">
          {/* 3D Attitude Indicator */}
          <Card className="glass lg:col-span-1 p-6 space-y-4">
            <h3 className="text-lg font-semibold flex items-center gap-2">
              <Navigation className="w-5 h-5 text-accent" />
              姿態指示器
            </h3>
            <div className="aspect-square rounded-lg bg-muted/20 flex items-center justify-center border border-border/50">
              <div className="space-y-2">
                <p className="text-sm text-muted-foreground">3D 可視化</p>
                <div className="space-y-1 text-sm">
                  <p>Pitch: {data!.attitude.pitch}°</p>
                  <p>Roll: {data!.attitude.roll}°</p>
                  <p>Yaw: {data!.attitude.yaw}°</p>
                </div>
              </div>
            </div>
          </Card>

          {/* Camera Feed */}
          <Card className="glass lg:col-span-2 p-6 space-y-4">
            <h3 className="text-lg font-semibold flex items-center gap-2">
              <Camera className="w-5 h-5 text-accent" />
              主攝影機實時畫面
            </h3>
            <div className="aspect-video rounded-lg bg-muted/20 flex items-center justify-center border border-border/50 relative overflow-hidden">
              {demoMode ? (
                <>
                  {/* Auto-play can be blocked; use ref and play() on canPlay to improve reliability */}
                  <VideoPlayer />
                  <div className="absolute top-4 left-4 space-y-2">
                    <Badge className="bg-destructive/90">● REC</Badge>
                    <Badge variant="outline" className="border-accent/50">目標識別中...</Badge>
                  </div>
                </>
              ) : (
                <div className="text-center space-y-2">
                  <div className="absolute top-4 left-4 space-y-2">
                    <Badge className="bg-destructive/90">● REC</Badge>
                    <Badge variant="outline" className="border-accent/50">目標識別中...</Badge>
                  </div>
                  <p className="text-sm text-muted-foreground">攝像頭畫面載入中...</p>
                </div>
              )}
            </div>
          </Card>
        </div>

        {/* Mission Progress */}
        <Card className="glass p-6 space-y-4 mt-6">
          <h3 className="text-lg font-semibold">任務進度</h3>
          <div className="space-y-4">
            <div className="flex items-center justify-between">
              <span className="text-sm">當前動作: 前進至目標區域</span>
              <Badge variant="outline" className="border-accent/50 text-accent">執行中</Badge>
            </div>
            <Progress value={missionProgress.percent} className="h-2" />
            <div className="grid grid-cols-3 gap-4 text-sm">
              <div>
                <p className="text-muted-foreground">已完成</p>
                <p className="font-semibold">{missionProgress.completed} / {missionProgress.total} 動作</p>
              </div>
              <div>
                <p className="text-muted-foreground">發現目標</p>
                <p className="font-semibold">{missionProgress.found} 次</p>
              </div>
              <div>
                <p className="text-muted-foreground">任務計時</p>
                <p className="font-semibold">{new Date(missionProgress.seconds * 1000).toISOString().substr(11, 8)}</p>
              </div>
            </div>
          </div>
        </Card>

        {/* Thruster Status */}
        <Card className="glass p-6 space-y-4 mt-6">
          <h3 className="text-lg font-semibold">推進器狀態</h3>
          <div className="grid grid-cols-4 gap-4">
            {thrustersRPM.map((rpm, index) => {
              const percent = Math.round((rpm / 2000) * 100);
              return (
                <div key={index} className="space-y-2">
                  <div className="flex items-center justify-between text-sm">
                    <span className="text-muted-foreground">推進器 {index + 1}</span>
                    <span className="font-medium">{rpm} RPM</span>
                  </div>
                  <Progress value={percent} className="h-1" />
                </div>
              );
            })}
          </div>
        </Card>
      </div>
    </div>
  );
}

export default Dashboard;
