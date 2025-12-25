import { Button } from "@/components/ui/button";
import { useNavigate } from "react-router-dom";
import { Target, Gauge, Waves } from "lucide-react";

const Index = () => {
  const navigate = useNavigate();

  return (
    <div className="min-h-screen bg-gradient-to-b from-background via-background to-primary/5">
      {/* Hero Section */}
      <div className="container mx-auto px-6 py-20">
        <div className="max-w-4xl mx-auto text-center space-y-8">
          {/* Logo */}
          <div className="flex justify-center">
            <div className="w-24 h-24 rounded-2xl bg-primary/20 flex items-center justify-center glow-blue border border-accent/30">
              <Waves className="w-12 h-12 text-accent" />
            </div>
          </div>

          {/* Title */}
          <div className="space-y-4">
            <h1 className="text-5xl md:text-6xl font-bold tracking-tight">
              <span className="bg-gradient-to-r from-accent via-primary to-accent bg-clip-text text-transparent">
                AUV 控制系統
              </span>
            </h1>
            <p className="text-xl text-muted-foreground max-w-2xl mx-auto">
              專業的自主水下航行器操作平台 - 智能目標追蹤、任務自動化、實時監控與手動遙控
            </p>
          </div>

          {/* CTA Buttons */}
          <div className="flex flex-wrap justify-center gap-4 pt-8">
            <Button
              size="lg"
              onClick={() => navigate("/mission-setup")}
              className="bg-accent hover:bg-accent/90 text-accent-foreground glow-cyan font-semibold"
            >
              <Target className="mr-2 h-5 w-5" />
              創建新任務
            </Button>
            <Button
              size="lg"
              variant="outline"
              onClick={() => navigate("/dashboard")}
              className="border-accent/50 hover:bg-accent/10"
            >
              <Gauge className="mr-2 h-5 w-5" />
              監控儀表板
            </Button>
          </div>

          {/* Features Grid */}
          <div className="grid md:grid-cols-3 gap-6 pt-16">
            <div className="glass rounded-xl p-6 space-y-3 hover:border-accent/50 transition-all">
              <div className="w-12 h-12 rounded-lg bg-accent/20 flex items-center justify-center">
                <Target className="w-6 h-6 text-accent" />
              </div>
              <h3 className="text-lg font-semibold">智能目標追蹤</h3>
              <p className="text-sm text-muted-foreground">
                上傳目標物體照片，AI 自動識別並追蹤
              </p>
            </div>

            <div className="glass rounded-xl p-6 space-y-3 hover:border-accent/50 transition-all">
              <div className="w-12 h-12 rounded-lg bg-primary/20 flex items-center justify-center">
                <Gauge className="w-6 h-6 text-primary-foreground" />
              </div>
              <h3 className="text-lg font-semibold">實時監控</h3>
              <p className="text-sm text-muted-foreground">
                3D 姿態顯示、深度計、電池狀態即時更新
              </p>
            </div>

            <div className="glass rounded-xl p-6 space-y-3 hover:border-accent/50 transition-all">
              <div className="w-12 h-12 rounded-lg bg-success/20 flex items-center justify-center">
                <Waves className="w-6 h-6 text-success" />
              </div>
              <h3 className="text-lg font-semibold">任務自動化</h3>
              <p className="text-sm text-muted-foreground">
                拖放式動作編輯器，輕鬆配置複雜任務序列
              </p>
            </div>
          </div>

          {/* Status Indicator */}
          <div className="pt-12">
            <div className="glass rounded-lg p-4 inline-flex items-center gap-3">
              <div className="w-3 h-3 rounded-full bg-success animate-pulse glow-cyan" />
              <span className="text-sm text-muted-foreground">系統就緒 | 等待連接</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Index;
