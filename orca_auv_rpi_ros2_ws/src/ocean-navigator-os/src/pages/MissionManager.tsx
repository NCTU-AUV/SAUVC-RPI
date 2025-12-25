import { useState } from "react";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Input } from "@/components/ui/input";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Plus, Search, Clock, Target, Edit, Copy, Trash2, Download } from "lucide-react";
import { toast } from "sonner";
import { useNavigate } from "react-router-dom";
import { useDemoMode } from "@/context/DemoModeContext";
import { demoMissionManagerData } from "@/data/demoData";

const MissionManager = () => {
  const { demoMode } = useDemoMode();
  const navigate = useNavigate();
  const [searchQuery, setSearchQuery] = useState("");
  const { missions, history } = demoMode ? demoMissionManagerData : { missions: [], history: [] };

  if (!demoMode) {
    return (
      <div className="min-h-screen p-6 space-y-6 flex items-center justify-center">
        <Card className="glass p-12 text-center max-w-md">
          <h1 className="text-2xl font-bold mb-4">任務管理</h1>
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
        <div className="flex items-center justify-between mb-8">
          <div>
            <h1 className="text-3xl font-bold">任務管理</h1>
            <p className="text-muted-foreground mt-1">創建、編輯和管理 AUV 任務</p>
          </div>
          <div className="flex items-center gap-3">
            {demoMode && (
              <Badge className="bg-accent/20 text-accent border border-accent/50">
                📺 展示模式
              </Badge>
            )}
            <Button
              className="bg-accent hover:bg-accent/90 glow-cyan"
              onClick={() => navigate("/mission-setup")}
            >
              <Plus className="mr-2 h-4 w-4" />
              創建新任務
            </Button>
          </div>
        </div>

        <Tabs defaultValue="missions" className="space-y-6">
          <TabsList className="glass">
            <TabsTrigger value="missions">任務列表</TabsTrigger>
            <TabsTrigger value="history">歷史記錄</TabsTrigger>
          </TabsList>

          <TabsContent value="missions" className="space-y-6">
            {/* Search */}
            <div className="relative">
              <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-muted-foreground" />
              <Input placeholder="搜索任務..." className="pl-10 glass" value={searchQuery} onChange={(e) => setSearchQuery(e.target.value)} />
            </div>

            {/* Mission Cards */}
            <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-4">
              {missions.map((mission) => (
                <Card key={mission.id} className="glass p-6 space-y-4 hover:border-accent/50 transition-all">
                  <div className="flex items-start justify-between">
                    <div className="space-y-1">
                      <h3 className="font-semibold">{mission.name}</h3>
                      <div className="flex items-center gap-2 text-xs text-muted-foreground">
                        <Clock className="w-3 h-3" />
                        <span>{mission.created}</span>
                      </div>
                    </div>
                    <Badge
                      variant="outline"
                      className={
                        mission.status === "已完成"
                          ? "border-success/50 text-success"
                          : mission.status === "執行中"
                          ? "border-accent/50 text-accent"
                          : "border-muted-foreground/50"
                      }
                    >
                      {mission.status}
                    </Badge>
                  </div>

                  <div className="flex items-center gap-4 text-sm">
                    <div className="flex items-center gap-1 text-muted-foreground">
                      <Target className="w-4 h-4" />
                      <span>{mission.actions} 動作</span>
                    </div>
                    <div className="flex items-center gap-1 text-muted-foreground">
                      <Clock className="w-4 h-4" />
                      <span>{mission.duration}</span>
                    </div>
                  </div>

                  <div className="flex gap-2 pt-2">
                    <Button
                      variant="outline"
                      size="sm"
                      className="flex-1"
                      onClick={() => navigate(`/mission-setup?id=${mission.id}`)}
                    >
                      <Edit className="mr-1 h-3 w-3" />
                      編輯
                    </Button>
                    <Button variant="outline" size="sm" onClick={() => toast.success(`已複製任務: ${mission.name}`)}>
                      <Copy className="h-3 w-3" />
                    </Button>
                    <Button variant="outline" size="sm" className="border-destructive/50 hover:bg-destructive/10" onClick={() => toast.error(`已刪除任務: ${mission.name}`)}>
                      <Trash2 className="h-3 w-3 text-destructive" />
                    </Button>
                  </div>
                </Card>
              ))}
            </div>
          </TabsContent>

          <TabsContent value="history" className="space-y-6">
            {/* History Cards */}
            <div className="space-y-4">
              {history.map((record) => (
                <Card key={record.id} className="glass p-6">
                  <div className="flex items-start justify-between mb-4">
                    <div>
                      <h3 className="font-semibold text-lg">{record.name}</h3>
                      <p className="text-sm text-muted-foreground mt-1">{record.date}</p>
                    </div>
                    <Button variant="outline" size="sm" className="border-accent/50" onClick={() => toast.success(`已下載報告: ${record.name}`)}>
                      <Download className="mr-2 h-4 w-4" />
                      下載報告
                    </Button>
                  </div>

                  <div className="grid grid-cols-4 gap-4 text-sm">
                    <div className="space-y-1">
                      <p className="text-muted-foreground">執行時長</p>
                      <p className="font-semibold">{record.duration}</p>
                    </div>
                    <div className="space-y-1">
                      <p className="text-muted-foreground">發現目標</p>
                      <p className="font-semibold">{record.targets} 次</p>
                    </div>
                    <div className="space-y-1">
                      <p className="text-muted-foreground">拍攝照片</p>
                      <p className="font-semibold">{record.photos} 張</p>
                    </div>
                    <div className="space-y-1">
                      <p className="text-muted-foreground">錄製影片</p>
                      <p className="font-semibold">{record.videos} 個</p>
                    </div>
                  </div>

                  <div className="flex gap-2 mt-4 pt-4 border-t border-border">
                    <Button variant="outline" size="sm" onClick={() => toast.info("正在載入軌跡回放...")}>
                      查看軌跡回放
                    </Button>
                    <Button variant="outline" size="sm" onClick={() => toast.success("已下載照片")}>
                      下載照片
                    </Button>
                    <Button variant="outline" size="sm" onClick={() => toast.success("已下載影片")}>
                      下載影片
                    </Button>
                    <Button variant="outline" size="sm" onClick={() => toast.success("已下載日誌")}>
                      下載日誌
                    </Button>
                  </div>
                </Card>
              ))}
            </div>
          </TabsContent>
        </Tabs>
      </div>
    </div>
  );
};

export default MissionManager;
