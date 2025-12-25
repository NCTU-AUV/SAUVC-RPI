// Demo 模式數據模擬器 - 模擬動態變化的 AUV 數據

export class DemoDataSimulator {
  private startTime: number;
  private baseDepth: number = 10;
  private baseTemp: number = 15;
  private baseBattery: number = 85;

  constructor() {
    this.startTime = Date.now();
  }

  /**
   * 取得模擬的深度數據（動態增加）
   */
  getDepth(): number {
    const elapsed = (Date.now() - this.startTime) / 1000; // 轉換為秒
    const depth = this.baseDepth + (elapsed * 0.5); // 每秒增加 0.5 米
    return Math.min(Math.round(depth * 10) / 10, 100); // 最多 100 米
  }

  /**
   * 取得模擬的水溫數據（浮動）
   */
  getTemperature(): number {
    const elapsed = (Date.now() - this.startTime) / 1000;
    const fluctuation = Math.sin(elapsed * 0.5) * 2; // ±2°C 浮動
    return Math.round((this.baseTemp + fluctuation) * 10) / 10;
  }

  /**
   * 取得模擬的電池百分比（緩慢下降）
   */
  getBattery(): number {
    const elapsed = (Date.now() - this.startTime) / 1000;
    const battery = this.baseBattery - (elapsed * 0.05); // 每秒減少 0.05%
    return Math.max(Math.round(battery * 10) / 10, 0);
  }

  /**
   * 取得模擬的水壓數據
   */
  getPressure(): number {
    const depth = this.getDepth();
    return Math.round((1.013 + depth * 0.1) * 100) / 100; // 大約每 10 米 1 bar
  }

  /**
   * 取得模擬的姿態數據（緩慢變化）
   */
  getAttitude(): { pitch: number; roll: number; yaw: number } {
    const elapsed = (Date.now() - this.startTime) / 1000;
    return {
      pitch: Math.round((Math.sin(elapsed * 0.3) * 8) * 10) / 10,
      roll: Math.round((Math.cos(elapsed * 0.4) * 5) * 10) / 10,
      yaw: Math.round((elapsed * 10) % 360 * 10) / 10
    };
  }

  /**
   * 取得模擬的任務進度（0-100%）
   */
  getTaskProgress(): number {
    const elapsed = (Date.now() - this.startTime) / 1000;
    const progress = (elapsed / 120) * 100; // 2 分鐘完成
    return Math.min(Math.round(progress), 100);
  }

  /**
   * 取得模擬的推進器轉速
   */
  getThrusterSpeeds(): number[] {
    const elapsed = (Date.now() - this.startTime) / 1000;
    return Array.from({ length: 8 }, (_, i) => {
      const speed = 1200 + Math.sin(elapsed * 0.5 + i) * 200;
      return Math.round(speed);
    });
  }

  /**
   * 取得模擬的發現目標數
   */
  getTargetCount(): number {
    const elapsed = (Date.now() - this.startTime) / 1000;
    return Math.min(Math.floor(elapsed / 15), 5); // 每 15 秒發現 1 個目標，最多 5 個
  }

  /**
   * 取得模擬的拍照數
   */
  getPhotoCount(): number {
    const elapsed = (Date.now() - this.startTime) / 1000;
    return Math.floor(elapsed / 5); // 每 5 秒拍 1 張，已經拍了多少張
  }

  /**
   * 取得模擬的任務時間
   */
  getTaskTime(): string {
    const elapsed = Math.floor((Date.now() - this.startTime) / 1000);
    const hours = Math.floor(elapsed / 3600);
    const minutes = Math.floor((elapsed % 3600) / 60);
    const seconds = elapsed % 60;
    return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
  }

  /**
   * 取得模擬的連接狀態（偶爾掉線）
   */
  getConnection(): { status: string; latency: number; bandwidth: string } {
    const elapsed = (Date.now() - this.startTime) / 1000;
    const isConnected = Math.sin(elapsed * 0.2) > -0.5; // 大約 75% 的時間連接正常
    const latency = 45 + Math.sin(elapsed * 0.5) * 10; // 45±10 ms
    
    return {
      status: isConnected ? 'connected' : 'unstable',
      latency: Math.round(latency),
      bandwidth: isConnected ? 'good' : 'poor'
    };
  }

  /**
   * 重置模擬器
   */
  reset(): void {
    this.startTime = Date.now();
  }

  /**
   * 取得完整的模擬數據
   */
  getFullData() {
    return {
      status: {
        battery: this.getBattery(),
        depth: this.getDepth(),
        pressure: this.getPressure(),
        temperature: this.getTemperature(),
        connection: this.getConnection().status,
        latency: this.getConnection().latency,
        bandwidth: this.getConnection().bandwidth
      },
      attitude: this.getAttitude(),
      taskProgress: {
        overall: this.getTaskProgress(),
        targets: this.getTargetCount(),
        photos: this.getPhotoCount(),
        taskTime: this.getTaskTime()
      },
      thrusters: this.getThrusterSpeeds()
    };
  }
}

// 建立全局示例
export const demoSimulator = new DemoDataSimulator();
