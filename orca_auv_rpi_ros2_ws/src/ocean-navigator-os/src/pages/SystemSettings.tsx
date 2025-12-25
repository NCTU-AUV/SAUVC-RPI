import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Button } from "@/components/ui/button";
import { Slider } from "@/components/ui/slider";
import { Switch } from "@/components/ui/switch";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Settings, Wifi, Shield, Bell, Save, Moon, Sun } from "lucide-react";
import { toast } from "sonner";
import { useDemoMode } from "@/context/DemoModeContext";
import { useTheme } from "@/hooks/useTheme";
import { demoSystemSettingsData } from "@/data/demoData";

// 使用 demo 資料中的 AUV 機型配置
const AUV_MODELS = demoSystemSettingsData.auvModels;

const SystemSettings = () => {
  const { demoMode, setDemoMode, testMode, setTestMode } = useDemoMode();
  const { theme, toggleTheme } = useTheme();
  const [deviceName, setDeviceName] = useState(demoSystemSettingsData.device.name);
  const [modelType, setModelType] = useState<keyof typeof AUV_MODELS>(demoSystemSettingsData.defaultModel);
  const [ipAddress, setIpAddress] = useState(demoSystemSettingsData.communication.ipAddress);
  const [port, setPort] = useState(demoSystemSettingsData.communication.port);
  const [maxDepth, setMaxDepth] = useState([AUV_MODELS[demoSystemSettingsData.defaultModel].maxDepth]);
  const [maxDistance, setMaxDistance] = useState([AUV_MODELS[demoSystemSettingsData.defaultModel].maxDistance]);
  const [batteryThreshold, setBatteryThreshold] = useState([20]);
  const [autoSurface, setAutoSurface] = useState(true);
  const [autoReturn, setAutoReturn] = useState(true);
  const [collisionDetection, setCollisionDetection] = useState(true);

  const currentModel = AUV_MODELS[modelType];

  const handleModelChange = (newModel: keyof typeof AUV_MODELS) => {
    setModelType(newModel);
    const model = AUV_MODELS[newModel];
    setMaxDepth([model.maxDepth]);
    setMaxDistance([model.maxDistance]);
    toast.success(`已切換至 ${model.name}`);
  };

  const handleSave = () => {
    toast.success("設置已保存");
  };

  return (
    <div className="min-h-screen p-6 space-y-6">
      <div className="max-w-5xl mx-auto">
        {/* Header */}
        <div className="flex items-center justify-between mb-8">
          <div>
            <h1 className="text-3xl font-bold">系統設置</h1>
            <p className="text-muted-foreground mt-1">配置 AUV 參數和安全設置</p>
          </div>
          <Button onClick={handleSave} className="bg-accent hover:bg-accent/90 glow-cyan">
            <Save className="mr-2 h-4 w-4" />
            保存設置
          </Button>
        </div>

        <Tabs defaultValue="auv" className="space-y-6">
          <TabsList className="glass grid w-full grid-cols-4">
            <TabsTrigger value="auv">
              <Settings className="mr-2 h-4 w-4" />
              AUV 配置
            </TabsTrigger>
            <TabsTrigger value="communication">
              <Wifi className="mr-2 h-4 w-4" />
              通訊參數
            </TabsTrigger>
            <TabsTrigger value="safety">
              <Shield className="mr-2 h-4 w-4" />
              安全設置
            </TabsTrigger>
            <TabsTrigger value="notifications">
              <Bell className="mr-2 h-4 w-4" />
              通知設置
            </TabsTrigger>
          </TabsList>

          {/* AUV Configuration */}
          <TabsContent value="auv" className="space-y-6">
            <Card className="glass p-6 space-y-6">
              <div>
                <h3 className="text-lg font-semibold mb-4">基本設置</h3>
                <div className="space-y-4">
                  <div className="flex items-center justify-between p-3 rounded-lg border border-accent/30 bg-accent/5">
                    <div>
                      <Label className="text-base font-semibold">展示模式</Label>
                      <p className="text-xs text-muted-foreground mt-1">
                        {demoMode ? "✓ 已啟用 - 使用本地演示數據" : "✗ 已禁用 - 連接真實 API 串口"}
                      </p>
                    </div>
                    <Switch
                      checked={demoMode}
                      onCheckedChange={(checked) => {
                        setDemoMode(checked);
                        toast.info(checked ? "已啟用展示模式，使用本地演示數據" : "已禁用展示模式，將連接真實 API 串口");
                      }}
                    />
                  </div>

                  <div className="flex items-center justify-between p-3 rounded-lg border border-warning/30 bg-warning/5">
                    <div>
                      <Label className="text-base font-semibold">測試模式</Label>
                      <p className="text-xs text-muted-foreground mt-1">
                        {testMode ? "✓ 已啟用 - 進入模擬測試狀態" : "✗ 已禁用 - 正常運作模式"}
                      </p>
                    </div>
                    <Switch
                      checked={testMode}
                      onCheckedChange={(checked) => {
                        setTestMode(checked);
                        toast.warning(checked ? "已啟用測試模式" : "已禁用測試模式");
                      }}
                    />
                  </div>

                  <div className="flex items-center justify-between p-3 rounded-lg border border-primary/30 bg-primary/5">
                    <div>
                      <Label className="text-base font-semibold">深色模式</Label>
                      <p className="text-xs text-muted-foreground mt-1">
                        {theme === "dark" ? "🌙 深色模式 - 夜間友善介面" : "☀️ 淺色模式 - 日間清晰介面"}
                      </p>
                    </div>
                    <Button
                      size="sm"
                      variant="outline"
                      onClick={() => {
                        toggleTheme();
                        toast.success(theme === "dark" ? "已切換至淺色模式" : "已切換至深色模式");
                      }}
                      className="gap-2"
                    >
                      {theme === "dark" ? (
                        <>
                          <Sun className="h-4 w-4" />
                          淺色
                        </>
                      ) : (
                        <>
                          <Moon className="h-4 w-4" />
                          深色
                        </>
                      )}
                    </Button>
                  </div>

                  {/* AUV Model Selection */}
                  <div className="space-y-3 p-4 rounded-lg border border-border bg-muted/20">
                    <Label className="text-base font-semibold">AUV 機型選擇</Label>
                    <Select value={modelType} onValueChange={(value) => handleModelChange(value as keyof typeof AUV_MODELS)}>
                      <SelectTrigger className="glass">
                        <SelectValue />
                      </SelectTrigger>
                      <SelectContent>
                        {Object.entries(AUV_MODELS).map(([key, model]) => (
                          <SelectItem key={key} value={key}>
                            {model.name} - 最大深度: {model.maxDepth}m
                          </SelectItem>
                        ))}
                      </SelectContent>
                    </Select>
                    <div className="grid grid-cols-2 gap-3 text-sm">
                      <div className="p-2 rounded bg-primary/10 border border-primary/30">
                        <p className="text-muted-foreground text-xs">最大深度</p>
                        <p className="font-semibold text-primary">{currentModel.maxDepth} 米</p>
                      </div>
                      <div className="p-2 rounded bg-accent/10 border border-accent/30">
                        <p className="text-muted-foreground text-xs">最大距離</p>
                        <p className="font-semibold text-accent">{currentModel.maxDistance} 米</p>
                      </div>
                      <div className="p-2 rounded bg-success/10 border border-success/30">
                        <p className="text-muted-foreground text-xs">最大速度</p>
                        <p className="font-semibold text-success">{currentModel.maxSpeed} m/s</p>
                      </div>
                      <div className="p-2 rounded bg-warning/10 border border-warning/30">
                        <p className="text-muted-foreground text-xs">電池容量</p>
                        <p className="font-semibold text-warning">{currentModel.battery} 小時</p>
                      </div>
                    </div>
                    <p className="text-xs text-muted-foreground bg-muted/50 p-2 rounded">
                      💡 {currentModel.description}
                    </p>
                  </div>

                  <div className="space-y-2">
                    <Label>設備名稱</Label>
                    <Input value={deviceName} onChange={(e) => setDeviceName(e.target.value)} />
                  </div>
                  <div className="space-y-2">
                    <Label>設備序列號</Label>
                    <Input defaultValue={demoSystemSettingsData.device.serialNumber} disabled />
                  </div>
                  <div className="space-y-2">
                    <Label>固件版本</Label>
                    <div className="flex gap-2">
                      <Input defaultValue={demoSystemSettingsData.device.firmwareVersion} disabled />
                      <Button variant="outline" onClick={() => toast.info("正在檢查更新...")}>檢查更新</Button>
                    </div>
                  </div>
                </div>
              </div>

              <div className="border-t border-border pt-6">
                <h3 className="text-lg font-semibold mb-4">傳感器校準</h3>
                <div className="grid grid-cols-2 gap-4">
                  <Button variant="outline" onClick={() => toast.success("深度計校準完成")}>深度計校準</Button>
                  <Button variant="outline" onClick={() => toast.success("羅盤校準完成")}>羅盤校準</Button>
                  <Button variant="outline" onClick={() => toast.success("加速度計校準完成")}>加速度計校準</Button>
                  <Button variant="outline" onClick={() => toast.success("陀螺儀校準完成")}>陀螺儀校準</Button>
                </div>
              </div>

              <div className="border-t border-border pt-6">
                <h3 className="text-lg font-semibold mb-4">推進器測試</h3>
                <div className="grid grid-cols-4 gap-2">
                  {[1, 2, 3, 4, 5, 6, 7, 8].map((i) => (
                    <Button key={i} variant="outline" size="sm" onClick={() => toast.info(`推進器 ${i} 測試中...`)}>
                      推進器 {i}
                    </Button>
                  ))}
                </div>
              </div>
            </Card>
          </TabsContent>

          {/* Communication Settings */}
          <TabsContent value="communication" className="space-y-6">
            <Card className="glass p-6 space-y-6">
              <div>
                <h3 className="text-lg font-semibold mb-4">網絡設置</h3>
                <div className="space-y-4">
                  <div className="space-y-2">
                    <Label>IP 地址</Label>
                    <Input value={ipAddress} onChange={(e) => setIpAddress(e.target.value)} />
                  </div>
                  <div className="space-y-2">
                    <Label>端口</Label>
                    <Input type="number" value={port} onChange={(e) => setPort(e.target.value)} />
                  </div>
                  <div className="space-y-2">
                    <Label>子網掩碼</Label>
                    <Input defaultValue="255.255.255.0" />
                  </div>
                  <div className="space-y-2">
                    <Label>網關</Label>
                    <Input defaultValue="192.168.1.1" />
                  </div>
                </div>
              </div>

              <div className="border-t border-border pt-6">
                <h3 className="text-lg font-semibold mb-4">連接測試</h3>
                <div className="flex gap-2">
                  <Button variant="outline" className="flex-1" onClick={() => toast.success("連接測試成功")}>測試連接</Button>
                  <Button variant="outline" className="flex-1" onClick={() => toast.success("Ping 正常 (45ms)")}>Ping 測試</Button>
                  <Button variant="outline" className="flex-1" onClick={() => toast.success("帶寬測試: 良好 (10Mbps)")}>帶寬測試</Button>
                </div>
              </div>
            </Card>
          </TabsContent>

          {/* Safety Settings */}
          <TabsContent value="safety" className="space-y-6">
            <Card className="glass p-6 space-y-6">
              <div className="p-3 rounded-lg border border-accent/30 bg-accent/5">
                <p className="text-sm">
                  <span className="font-semibold">當前機型:</span> {currentModel.name} •
                  <span className="text-accent font-semibold ml-2">最大深度: {currentModel.maxDepth} 米</span>
                </p>
              </div>

              <div>
                <h3 className="text-lg font-semibold mb-4">操作限制</h3>
                <div className="space-y-6">
                  <div className="space-y-3">
                    <div className="flex items-center justify-between">
                      <div>
                        <Label>最大深度限制</Label>
                        <p className="text-xs text-muted-foreground mt-1">不可超過機型限制 ({currentModel.maxDepth} 米)</p>
                      </div>
                      <span className="text-sm font-medium text-accent">{maxDepth[0]} 米</span>
                    </div>
                    <Slider
                      value={maxDepth}
                      onValueChange={setMaxDepth}
                      min={0}
                      max={currentModel.maxDepth}
                      step={Math.max(1, Math.floor(currentModel.maxDepth / 20))}
                    />
                  </div>

                  <div className="space-y-3">
                    <div className="flex items-center justify-between">
                      <div>
                        <Label>最大距離限制</Label>
                        <p className="text-xs text-muted-foreground mt-1">不可超過機型限制 ({currentModel.maxDistance} 米)</p>
                      </div>
                      <span className="text-sm font-medium text-accent">{maxDistance[0]} 米</span>
                    </div>
                    <Slider
                      value={maxDistance}
                      onValueChange={setMaxDistance}
                      min={0}
                      max={currentModel.maxDistance}
                      step={Math.max(50, Math.floor(currentModel.maxDistance / 20))}
                    />
                  </div>

                  <div className="space-y-3">
                    <div className="flex items-center justify-between">
                      <Label>電量返回閾值</Label>
                      <span className="text-sm font-medium text-accent">{batteryThreshold[0]}%</span>
                    </div>
                    <Slider value={batteryThreshold} onValueChange={setBatteryThreshold} min={10} max={50} step={5} />
                  </div>
                </div>
              </div>

              <div className="border-t border-border pt-6">
                <h3 className="text-lg font-semibold mb-4">緊急處理</h3>
                <div className="space-y-4">
                  <div className="flex items-center justify-between p-3 rounded-lg border border-border">
                    <div>
                      <Label>失聯自動上浮</Label>
                      <p className="text-xs text-muted-foreground mt-1">失去連接 5 分鐘後自動返回水面</p>
                    </div>
                    <Switch checked={autoSurface} onCheckedChange={setAutoSurface} />
                  </div>
                  <div className="flex items-center justify-between p-3 rounded-lg border border-border">
                    <div>
                      <Label>低電量自動返航</Label>
                      <p className="text-xs text-muted-foreground mt-1">電量低於閾值時自動返回</p>
                    </div>
                    <Switch checked={autoReturn} onCheckedChange={setAutoReturn} />
                  </div>
                  <div className="flex items-center justify-between p-3 rounded-lg border border-border">
                    <div>
                      <Label>碰撞檢測</Label>
                      <p className="text-xs text-muted-foreground mt-1">檢測到碰撞時自動停止</p>
                    </div>
                    <Switch checked={collisionDetection} onCheckedChange={setCollisionDetection} />
                  </div>
                </div>
              </div>
            </Card>
          </TabsContent>

          {/* Notification Settings */}
          <TabsContent value="notifications" className="space-y-6">
            <Card className="glass p-6 space-y-4">
              <h3 className="text-lg font-semibold mb-4">通知偏好</h3>
              <div className="space-y-4">
                <div className="flex items-center justify-between p-3 rounded-lg border border-border">
                  <div>
                    <Label>任務完成通知</Label>
                    <p className="text-xs text-muted-foreground mt-1">任務執行完成時發送通知</p>
                  </div>
                  <Switch defaultChecked onCheckedChange={(checked) => toast.info(checked ? "已啟用任務完成通知" : "已禁用任務完成通知")} />
                </div>
                <div className="flex items-center justify-between p-3 rounded-lg border border-border">
                  <div>
                    <Label>異常警報</Label>
                    <p className="text-xs text-muted-foreground mt-1">檢測到異常情況時立即通知</p>
                  </div>
                  <Switch defaultChecked onCheckedChange={(checked) => toast.info(checked ? "已啟用異常警報" : "已禁用異常警報")} />
                </div>
                <div className="flex items-center justify-between p-3 rounded-lg border border-border">
                  <div>
                    <Label>電量警告</Label>
                    <p className="text-xs text-muted-foreground mt-1">電量低於 30% 時發送警告</p>
                  </div>
                  <Switch defaultChecked onCheckedChange={(checked) => toast.info(checked ? "已啟用電量警告" : "已禁用電量警告")} />
                </div>
                <div className="flex items-center justify-between p-3 rounded-lg border border-border">
                  <div>
                    <Label>目標發現提醒</Label>
                    <p className="text-xs text-muted-foreground mt-1">成功識別目標時通知</p>
                  </div>
                  <Switch defaultChecked onCheckedChange={(checked) => toast.info(checked ? "已啟用目標發現提醒" : "已禁用目標發現提醒")} />
                </div>
              </div>
            </Card>
          </TabsContent>
        </Tabs>
      </div>
    </div>
  );
};

export default SystemSettings;
