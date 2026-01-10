class ROS2DMapViewer {
    constructor() {
        this.viewer = null;
        this.ros = null;
        this.gridClient = null;
        this.isConnected = false;

        // 订阅器
        this.lidarSubscription = null;
        this.poseSubscription = null;

        // 容器
        this.lidarContainer = null;
        this.robotContainer = null;

        // 状态
        this.hasMap = false;
        this.hasLidar = false;
        this.hasPose = false;

        // 机器人位姿
        this.robotPose = {
            x: 0,
            y: 0,
            theta: 0
        };

        // 地图信息
        this.mapInfo = {
            width: 0,
            height: 0,
            resolution: 0.05,
            origin: { x: 0, y: 0, theta: 0 }
        };

        // 显示控制
        this.showLidar = true;
        this.showRobot = true;

        // 话题监视器
        this.monitoredTopics = new Map(); // topicName -> { subscription, drawerId, data }
        this.topicTypes = new Map(); // topicName -> topicType
        this.drawerCounter = 0;

        // 初始化
        this.init();
    }

    init() {
        this.initViewer();
        this.initUI();
        this.initEventListeners();
        this.logMessage("系统初始化完成", "info");
    }

    // 初始化地图查看器
    initViewer() {
        try {
            this.viewer = new ROS2D.Viewer({
                divID: 'map',
                width: document.querySelector('.map-container').offsetWidth,
                height: document.querySelector('.map-container').offsetHeight
            });

            // 启用鼠标交互
            this.viewer.scale = 1.0;

            // 创建容器用于雷达和机器人显示
            this.lidarContainer = new createjs.Container();
            this.robotContainer = new createjs.Container();

            // 添加到场景
            this.viewer.scene.addChild(this.lidarContainer);
            this.viewer.scene.addChild(this.robotContainer);

            this.logMessage("地图查看器初始化成功", "success");
        } catch (error) {
            this.logMessage(`地图查看器初始化失败: ${error}`, "error");
        }
    }

    // 初始化UI
    initUI() {
        this.updateStatus('rosStatus', '未连接', 'disconnected');
        this.updateStatus('mapStatus', '无');
        this.updateStatus('lidarStatus', '无');
        this.updateStatus('poseStatus', '无');
    }

    // 连接ROS
    connectROS() {
        const url = document.getElementById('rosUrl').value;

        if (!url) {
            this.showNotification("请输入ROS Bridge URL", "warning");
            return;
        }

        this.logMessage(`正在连接到: ${url}`, "info");
        this.updateStatus('rosStatus', '连接中...');

        this.ros = new ROSLIB.Ros({
            url: url
        });

        this.ros.on('connection', () => {
            this.isConnected = true;
            this.updateStatus('rosStatus', '已连接', 'connected');
            document.getElementById('connectBtn').disabled = true;
            document.getElementById('disconnectBtn').disabled = false;
            document.getElementById('subscribeBtn').disabled = false;
            document.getElementById('showLidarBtn').disabled = false;
            document.getElementById('showRobotBtn').disabled = false;
            document.getElementById('refreshTopicsBtn').disabled = false;
            document.getElementById('addTopicBtn').disabled = false;

            this.showNotification("成功连接到ROS", "success");
            this.logMessage("ROS连接成功", "success");

            // 连接成功后刷新话题列表
            this.refreshTopicList();
        });

        this.ros.on('error', (error) => {
            this.logMessage(`连接错误: ${error}`, "error");
            this.updateStatus('rosStatus', '连接失败', 'disconnected');
            this.showNotification("连接失败，请检查URL和ROS服务", "error");
        });

        this.ros.on('close', () => {
            this.isConnected = false;
            this.updateStatus('rosStatus', '未连接', 'disconnected');
            document.getElementById('connectBtn').disabled = false;
            document.getElementById('disconnectBtn').disabled = true;
            document.getElementById('subscribeBtn').disabled = true;
            document.getElementById('showLidarBtn').disabled = true;
            document.getElementById('showRobotBtn').disabled = true;
            document.getElementById('refreshTopicsBtn').disabled = true;
            document.getElementById('addTopicBtn').disabled = true;

            // 清空话题列表
            this.clearTopicList();

            // 移除所有话题监视器
            this.removeAllMonitoredTopics();

            this.logMessage("ROS连接已关闭", "warning");
        });
    }

    // 断开ROS连接
    disconnectROS() {
        if (this.ros) {
            this.unsubscribeAll();
            this.ros.close();
            this.ros = null;
        }
    }

    // 刷新话题列表
    refreshTopicList() {
        if (!this.isConnected || !this.ros) {
            this.showNotification("未连接到ROS", "warning");
            return;
        }

        this.logMessage("获取话题列表中...", "info");

        // 通过ROS API获取话题列表
        const topicsClient = new ROSLIB.Service({
            ros: this.ros,
            name: '/rosapi/topics',
            serviceType: 'rosapi/GetTopics'
        });

        const request = new ROSLIB.ServiceRequest({});

        topicsClient.callService(request, (result) => {
            const topicSelector = document.getElementById('topicSelector');
            topicSelector.innerHTML = '<option value="">选择话题...</option>';

            if (result && result.topics && result.topics.length > 0) {
                // 清空之前的话题类型缓存
                this.topicTypes.clear();

                // 将话题和类型配对
                for (let i = 0; i < result.topics.length; i++) {
                    const topic = result.topics[i];
                    const type = result.types[i];

                    // 保存话题类型
                    this.topicTypes.set(topic, type);

                    // 添加到下拉列表
                    const option = document.createElement('option');
                    option.value = topic;
                    option.textContent = `${topic} (${type})`;
                    topicSelector.appendChild(option);
                }

                this.logMessage(`发现 ${result.topics.length} 个话题`, "success");
                this.showNotification(`发现 ${result.topics.length} 个话题`, "success");
            } else {
                this.logMessage("未找到可用话题", "warning");
                this.showNotification("未找到可用话题", "warning");
            }
        }, (error) => {
            this.logMessage(`获取话题列表失败: ${error}`, "error");
            this.showNotification("获取话题列表失败", "error");
        });
    }

    // 清空话题列表
    clearTopicList() {
        const topicSelector = document.getElementById('topicSelector');
        topicSelector.innerHTML = '<option value="">选择话题...</option>';
        this.topicTypes.clear();
    }

    // 添加话题监视器
    addTopicMonitor() {
        const topicSelector = document.getElementById('topicSelector');
        const selectedTopic = topicSelector.value;

        if (!selectedTopic) {
            this.showNotification("请选择一个话题", "warning");
            return;
        }

        if (this.monitoredTopics.has(selectedTopic)) {
            this.showNotification("该话题已在监视列表中", "warning");
            return;
        }

        const topicType = this.topicTypes.get(selectedTopic);
        if (!topicType) {
            this.logMessage(`未知的话题类型: ${selectedTopic}`, "error");
            return;
        }

        this.logMessage(`开始监视话题: ${selectedTopic}`, "info");

        // 创建话题订阅
        const subscription = new ROSLIB.Topic({
            ros: this.ros,
            name: selectedTopic,
            messageType: topicType
        });

        const drawerId = `drawer-${++this.drawerCounter}`;

        // 创建抽屉UI
        this.createTopicDrawer(selectedTopic, topicType, drawerId);

        // 订阅话题
        const startTime = Date.now();
        let messageCount = 0;

        subscription.subscribe((message) => {
            messageCount++;

            // 更新抽屉内容
            this.updateTopicDrawer(drawerId, selectedTopic, message, messageCount, startTime);
        });

        // 保存订阅信息
        this.monitoredTopics.set(selectedTopic, {
            subscription: subscription,
            drawerId: drawerId,
            messageCount: 0,
            startTime: startTime,
            lastMessage: null
        });

        this.showNotification(`已开始监视话题: ${selectedTopic}`, "success");
    }

    // 创建话题抽屉
    createTopicDrawer(topicName, topicType, drawerId) {
        const topicDrawers = document.getElementById('topicDrawers');

        const drawer = document.createElement('div');
        drawer.className = 'drawer';
        drawer.id = drawerId;

        drawer.innerHTML = `
          <div class="drawer-header">
            <div class="drawer-title">
              <span>📡</span>
              <span>话题监视</span>
              <span class="drawer-topic">${topicName}</span>
            </div>
            <div class="drawer-controls">
              <button class="drawer-toggle" data-drawer="${drawerId}">展开</button>
              <button class="drawer-remove" data-drawer="${drawerId}">×</button>
            </div>
          </div>
          <div class="drawer-content" id="${drawerId}-content">
            <div class="drawer-info">
              <div class="drawer-info-item">
                <span class="info-label">话题名称:</span>
                <span class="info-value" id="${drawerId}-name">${topicName}</span>
              </div>
              <div class="drawer-info-item">
                <span class="info-label">消息类型:</span>
                <span class="info-value" id="${drawerId}-type">${topicType}</span>
              </div>
              <div class="drawer-info-item">
                <span class="info-label">消息频率:</span>
                <span class="info-value" id="${drawerId}-frequency">0 Hz</span>
              </div>
              <div class="drawer-info-item">
                <span class="info-label">消息数量:</span>
                <span class="info-value" id="${drawerId}-count">0</span>
              </div>
            </div>
            <div class="message-data" id="${drawerId}-data">
              <div class="message-empty">等待消息...</div>
            </div>
          </div>
        `;

        // 添加到顶部
        topicDrawers.insertBefore(drawer, topicDrawers.firstChild);

        // 添加事件监听器
        const toggleBtn = drawer.querySelector('.drawer-toggle');
        const removeBtn = drawer.querySelector('.drawer-remove');

        toggleBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            this.toggleDrawer(drawerId);
        });

        removeBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            this.removeTopicMonitor(topicName);
        });

        // 点击头部也可以切换
        const drawerHeader = drawer.querySelector('.drawer-header');
        drawerHeader.addEventListener('click', (e) => {
            if (e.target !== toggleBtn && e.target !== removeBtn) {
                this.toggleDrawer(drawerId);
            }
        });
    }

    // 更新话题抽屉
    updateTopicDrawer(drawerId, topicName, message, messageCount, startTime) {
        const drawer = this.monitoredTopics.get(topicName);
        if (!drawer) return;

        drawer.messageCount = messageCount;
        drawer.lastMessage = message;

        // 计算消息频率
        const elapsedTime = (Date.now() - startTime) / 1000; // 秒
        const frequency = elapsedTime > 0 ? (messageCount / elapsedTime).toFixed(2) : "0";

        // 更新UI
        const frequencyElement = document.getElementById(`${drawerId}-frequency`);
        const countElement = document.getElementById(`${drawerId}-count`);
        const dataElement = document.getElementById(`${drawerId}-data`);

        if (frequencyElement) frequencyElement.textContent = `${frequency} Hz`;
        if (countElement) countElement.textContent = messageCount;

        // 显示消息内容
        if (dataElement) {
            // 尝试格式化消息
            let messageContent = '';
            try {
                if (typeof message === 'object') {
                    messageContent = JSON.stringify(message, null, 2);
                } else {
                    messageContent = String(message);
                }
            } catch (e) {
                messageContent = '无法解析消息内容';
            }

            dataElement.innerHTML = `<pre style="margin: 0; font-size: 11px; white-space: pre-wrap; word-wrap: break-word;">${this.escapeHtml(messageContent)}</pre>`;
        }

        // 更新抽屉中的最后更新时间
        const drawerElement = document.getElementById(drawerId);
        if (drawerElement) {
            const titleElement = drawerElement.querySelector('.drawer-title span:first-child');
            if (titleElement) {
                titleElement.textContent = '🔴'; // 活跃状态
                // 3秒后恢复
                setTimeout(() => {
                    if (titleElement) {
                        titleElement.textContent = '📡';
                    }
                }, 3000);
            }
        }
    }

    // 切换抽屉展开/收起
    toggleDrawer(drawerId) {
        const content = document.getElementById(`${drawerId}-content`);
        const toggleBtn = document.querySelector(`[data-drawer="${drawerId}"].drawer-toggle`);

        if (content && toggleBtn) {
            content.classList.toggle('expanded');
            toggleBtn.textContent = content.classList.contains('expanded') ? '收起' : '展开';
        }
    }

    // 移除话题监视器
    removeTopicMonitor(topicName) {
        const monitor = this.monitoredTopics.get(topicName);
        if (monitor) {
            // 取消订阅
            if (monitor.subscription) {
                monitor.subscription.unsubscribe();
            }

            // 移除UI
            const drawerElement = document.getElementById(monitor.drawerId);
            if (drawerElement) {
                drawerElement.remove();
            }

            // 从映射中移除
            this.monitoredTopics.delete(topicName);

            this.logMessage(`已停止监视话题: ${topicName}`, "info");
            this.showNotification(`已停止监视话题: ${topicName}`, "info");
        }
    }

    // 移除所有话题监视器
    removeAllMonitoredTopics() {
        for (const [topicName] of this.monitoredTopics) {
            this.removeTopicMonitor(topicName);
        }
        this.monitoredTopics.clear();
    }

    // HTML转义
    escapeHtml(text) {
        const map = {
            '&': '&amp;',
            '<': '&lt;',
            '>': '&gt;',
            '"': '&quot;',
            "'": '&#039;'
        };
        return text.replace(/[&<>"']/g, m => map[m]);
    }

    // 订阅所有话题
    subscribeAll() {
        this.subscribeMap();
        this.subscribeLidar();
        this.subscribePose();
    }

    // 订阅地图
    subscribeMap() {
        if (!this.isConnected || !this.ros) {
            this.showNotification("未连接到ROS", "warning");
            return;
        }

        const topicName = document.getElementById('mapTopic').value || '/map';

        this.logMessage(`订阅地图话题: ${topicName}`, "info");

        // 清除现有地图客户端
        if (this.gridClient) {
            this.viewer.scene.removeChild(this.gridClient.rootObject);
        }

        // 创建新的地图客户端
        this.gridClient = new ROS2D.OccupancyGridClient({
            ros: this.ros,
            rootObject: this.viewer.scene,
            continuous: true
        });

        this.gridClient.on('change', () => {
            this.hasMap = true;
            this.updateStatus('mapStatus', '有');

            if (this.gridClient.currentGrid) {
                this.mapInfo = {
                    width: this.gridClient.currentGrid.width,
                    height: this.gridClient.currentGrid.height,
                    resolution: this.gridClient.currentGrid.resolution,
                    origin: {
                        x: this.gridClient.currentGrid.pose.position.x,
                        y: this.gridClient.currentGrid.pose.position.y,
                        theta: 0
                    }
                };

                this.logMessage(`地图加载: ${this.mapInfo.width}x${this.mapInfo.height}, 分辨率: ${this.mapInfo.resolution}`, "success");

                // 调整视图以适应地图
                this.viewer.scaleToDimensions(this.mapInfo.width, this.mapInfo.height);
                this.viewer.shift(this.mapInfo.origin.x, this.mapInfo.origin.y);

                this.showNotification("地图加载成功", "success");
            }
        });
    }

    // 订阅雷达
    subscribeLidar() {
        if (!this.isConnected || !this.ros) {
            this.showNotification("未连接到ROS", "warning");
            return;
        }

        const topicName = document.getElementById('lidarTopic').value || '/scan';

        this.logMessage(`订阅雷达话题: ${topicName}`, "info");

        // 取消现有订阅
        if (this.lidarSubscription) {
            this.lidarSubscription.unsubscribe();
        }

        this.lidarSubscription = new ROSLIB.Topic({
            ros: this.ros,
            name: topicName,
            messageType: 'sensor_msgs/msg/LaserScan'
        });

        this.lidarSubscription.subscribe((msg) => {
            this.hasLidar = true;
            this.updateStatus('lidarStatus', '有');
            this.updateLidarDisplay(msg);
        });
    }

    // 订阅机器人位姿
    subscribePose() {
        if (!this.isConnected || !this.ros) {
            this.showNotification("未连接到ROS", "warning");
            return;
        }

        const topicName = document.getElementById('poseTopic').value || '/amcl_pose';

        this.logMessage(`订阅位姿话题: ${topicName}`, "info");

        // 取消现有订阅
        if (this.poseSubscription) {
            this.poseSubscription.unsubscribe();
        }

        this.poseSubscription = new ROSLIB.Topic({
            ros: this.ros,
            name: topicName,
            messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped'
        });

        this.poseSubscription.subscribe((msg) => {
            this.hasPose = true;
            this.updateStatus('poseStatus', '有');
            this.updateRobotPose(msg);
        });
    }

    // 更新雷达显示
    updateLidarDisplay(scanMsg) {
        if (!this.showLidar) return;

        // 清除之前的雷达点
        this.lidarContainer.removeAllChildren();

        const ranges = scanMsg.ranges;
        const angleMin = scanMsg.angle_min;
        const angleIncrement = scanMsg.angle_increment;

        if (!ranges || ranges.length === 0) return;

        // 创建雷达点图形
        const points = new createjs.Shape();
        const g = points.graphics;

        g.beginFill(createjs.Graphics.getRGB(255, 0, 0, 0.7));

        // 计算每个雷达点在地图坐标系中的位置
        for (let i = 0; i < ranges.length; i++) {
            const range = ranges[i];

            // 跳过无效点
            if (!isFinite(range) || range <= 0 || range > scanMsg.range_max) {
                continue;
            }

            const angle = angleMin + i * angleIncrement;

            // 雷达在机器人坐标系中的坐标
            const localX = Math.cos(angle) * range;
            const localY = Math.sin(angle) * range;

            // 转换到世界坐标系（考虑机器人位姿）
            const cosTheta = Math.cos(this.robotPose.theta);
            const sinTheta = Math.sin(this.robotPose.theta);

            const rotatedX = localX * cosTheta - localY * sinTheta;
            const rotatedY = localX * sinTheta + localY * cosTheta;

            const worldX = this.robotPose.x + rotatedX;
            const worldY = this.robotPose.y + rotatedY;

            // 转换到像素坐标（考虑地图分辨率）
            const pixelsPerMeter = 1 / this.mapInfo.resolution;
            const pixelX = worldX * pixelsPerMeter;
            const pixelY = worldY * pixelsPerMeter;

            // 考虑地图原点偏移
            const finalX = pixelX - (this.mapInfo.origin.x * pixelsPerMeter);
            const finalY = pixelY - (this.mapInfo.origin.y * pixelsPerMeter);

            // 绘制点
            g.drawCircle(finalX, finalY, 2);
        }

        g.endFill();
        this.lidarContainer.addChild(points);
    }

    // 更新机器人位姿
    updateRobotPose(poseMsg) {
        if (!this.showRobot) return;

        // 清除之前的机器人标记
        this.robotContainer.removeAllChildren();

        const pose = poseMsg.pose.pose;
        this.robotPose.x = pose.position.x;
        this.robotPose.y = pose.position.y;

        // 从四元数计算偏航角
        const q = pose.orientation;
        this.robotPose.theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

        // 转换到像素坐标
        const pixelsPerMeter = 1 / this.mapInfo.resolution;
        const robotX = this.robotPose.x * pixelsPerMeter - (this.mapInfo.origin.x * pixelsPerMeter);
        const robotY = this.robotPose.y * pixelsPerMeter - (this.mapInfo.origin.y * pixelsPerMeter);

        // 创建机器人标记
        const robot = new createjs.Shape();
        robot.graphics
            .setStrokeStyle(2)
            .beginStroke("#00FF00")
            .drawCircle(robotX, robotY, 10)
            .moveTo(robotX, robotY)
            .lineTo(robotX + Math.cos(this.robotPose.theta) * 20, robotY + Math.sin(this.robotPose.theta) * 20);

        this.robotContainer.addChild(robot);
    }

    // 取消所有订阅
    unsubscribeAll() {
        if (this.lidarSubscription) {
            this.lidarSubscription.unsubscribe();
            this.lidarSubscription = null;
        }

        if (this.poseSubscription) {
            this.poseSubscription.unsubscribe();
            this.poseSubscription = null;
        }

        if (this.gridClient) {
            this.viewer.scene.removeChild(this.gridClient.rootObject);
            this.gridClient = null;
        }

        this.hasMap = false;
        this.hasLidar = false;
        this.hasPose = false;

        this.updateStatus('mapStatus', '无');
        this.updateStatus('lidarStatus', '无');
        this.updateStatus('poseStatus', '无');

        this.lidarContainer.removeAllChildren();
        this.robotContainer.removeAllChildren();

        this.logMessage("已取消所有订阅", "info");
    }

    // 重置视图
    resetView() {
        if (this.gridClient && this.gridClient.currentGrid) {
            this.viewer.scaleToDimensions(this.gridClient.currentGrid.width, this.gridClient.currentGrid.height);
            this.viewer.shift(this.gridClient.currentGrid.pose.position.x, this.gridClient.currentGrid.pose.position.y);
            this.logMessage("视图已重置", "info");
        } else {
            this.showNotification("没有地图数据，无法重置视图", "warning");
        }
    }

    // 显示/隐藏雷达
    toggleLidar() {
        this.showLidar = !this.showLidar;
        this.lidarContainer.visible = this.showLidar;

        if (this.showLidar) {
            document.getElementById('showLidarBtn').textContent = '隐藏雷达';
            this.logMessage("雷达显示已开启", "info");
        } else {
            document.getElementById('showLidarBtn').textContent = '显示雷达';
            this.logMessage("雷达显示已关闭", "info");
        }
    }

    // 显示/隐藏机器人
    toggleRobot() {
        this.showRobot = !this.showRobot;
        this.robotContainer.visible = this.showRobot;

        if (this.showRobot) {
            document.getElementById('showRobotBtn').textContent = '隐藏机器人';
            this.logMessage("机器人显示已开启", "info");
        } else {
            document.getElementById('showRobotBtn').textContent = '显示机器人';
            this.logMessage("机器人显示已关闭", "info");
        }
    }

    // 更新状态显示
    updateStatus(elementId, text, className = '') {
        const element = document.getElementById(elementId);
        element.textContent = text;
        element.className = 'status-value ' + className;
    }

    // 日志消息
    logMessage(message, type = "info") {
        const logContent = document.getElementById('logContent');
        const timestamp = new Date().toLocaleTimeString();
        const entry = document.createElement('div');
        entry.className = 'log-entry';
        entry.innerHTML = `
          <span class="log-timestamp">[${timestamp}]</span>
          <span class="log-message">${message}</span>
        `;

        // 根据消息类型设置颜色
        if (type === "error") {
            entry.style.color = "#e74c3c";
        } else if (type === "success") {
            entry.style.color = "#2ecc71";
        } else if (type === "warning") {
            entry.style.color = "#f39c12";
        }

        logContent.insertBefore(entry, logContent.firstChild);

        // 限制日志数量
        while (logContent.children.length > 50) {
            logContent.removeChild(logContent.lastChild);
        }
    }

    // 显示通知
    showNotification(message, type = "info") {
        const notification = document.getElementById('notification');
        notification.textContent = message;
        notification.className = 'notification';

        if (type === "error") {
            notification.classList.add('error');
        } else if (type === "warning") {
            notification.classList.add('warning');
        }

        notification.classList.add('show');

        // 3秒后自动隐藏
        setTimeout(() => {
            notification.classList.remove('show');
        }, 3000);
    }

    // 初始化事件监听器
    initEventListeners() {
        // ROS连接按钮
        document.getElementById('connectBtn').addEventListener('click', () => {
            this.connectROS();
        });

        document.getElementById('disconnectBtn').addEventListener('click', () => {
            this.disconnectROS();
        });

        // 订阅按钮
        document.getElementById('subscribeBtn').addEventListener('click', () => {
            this.subscribeAll();
        });

        document.getElementById('unsubscribeBtn').addEventListener('click', () => {
            this.unsubscribeAll();
        });

        // 显示控制按钮
        document.getElementById('showLidarBtn').addEventListener('click', () => {
            this.toggleLidar();
        });

        document.getElementById('showRobotBtn').addEventListener('click', () => {
            this.toggleRobot();
        });

        document.getElementById('resetViewBtn').addEventListener('click', () => {
            this.resetView();
        });

        // 话题监视器按钮
        document.getElementById('refreshTopicsBtn').addEventListener('click', () => {
            this.refreshTopicList();
        });

        document.getElementById('addTopicBtn').addEventListener('click', () => {
            this.addTopicMonitor();
        });

        // 控制面板收起/展开
        document.getElementById('togglePanelBtn').addEventListener('click', () => {
            const controlPanel = document.getElementById('controlPanel');
            const toggleIcon = document.getElementById('toggleIcon');

            controlPanel.classList.toggle('collapsed');

            if (controlPanel.classList.contains('collapsed')) {
                toggleIcon.textContent = '▶';
                this.logMessage("控制面板已收起", "info");
            } else {
                toggleIcon.textContent = '◀';
                this.logMessage("控制面板已展开", "info");
            }

            // 调整地图大小
            setTimeout(() => {
                if (this.viewer) {
                    this.viewer.width = document.querySelector('.map-container').offsetWidth;
                    this.viewer.height = document.querySelector('.map-container').offsetHeight;
                    this.viewer.stage.canvas.width = this.viewer.width;
                    this.viewer.stage.canvas.height = this.viewer.height;
                }
            }, 300);
        });

        // 日志面板控制
        document.getElementById('toggleLogBtn').addEventListener('click', () => {
            const logPanel = document.getElementById('logPanel');
            const toggleBtn = document.getElementById('toggleLogBtn');

            logPanel.classList.toggle('visible');

            if (logPanel.classList.contains('visible')) {
                toggleBtn.textContent = '隐藏日志';
                this.logMessage("日志面板已显示", "info");
            } else {
                toggleBtn.textContent = '显示日志';
                this.logMessage("日志面板已隐藏", "info");
            }
        });

        // 清空日志
        document.getElementById('clearLogBtn').addEventListener('click', () => {
            document.getElementById('logContent').innerHTML = '';
            this.logMessage("日志已清空", "info");
        });

        // 全屏模式
        document.getElementById('fullscreenBtn').addEventListener('click', () => {
            if (!document.fullscreenElement) {
                document.documentElement.requestFullscreen().catch(err => {
                    this.logMessage(`全屏模式失败: ${err.message}`, "error");
                });
            } else {
                if (document.exitFullscreen) {
                    document.exitFullscreen();
                }
            }
        });

        // 帮助面板控制
        let helpVisible = true;
        document.getElementById('toggleHelpBtn').addEventListener('click', () => {
            const helpPanel = document.querySelector('.help-panel');
            const toggleBtn = document.getElementById('toggleHelpBtn');

            if (helpVisible) {
                helpPanel.style.display = 'none';
                toggleBtn.textContent = '显示提示';
                this.logMessage("操作提示已隐藏", "info");
            } else {
                helpPanel.style.display = 'block';
                toggleBtn.textContent = '隐藏提示';
                this.logMessage("操作提示已显示", "info");
            }

            helpVisible = !helpVisible;
        });

        // 窗口大小变化
        window.addEventListener('resize', () => {
            if (this.viewer) {
                this.viewer.width = document.querySelector('.map-container').offsetWidth;
                this.viewer.height = document.querySelector('.map-container').offsetHeight;
                this.viewer.stage.canvas.width = this.viewer.width;
                this.viewer.stage.canvas.height = this.viewer.height;
            }
        });

        // 键盘快捷键
        document.addEventListener('keydown', (e) => {
            switch (e.key.toLowerCase()) {
                case 'r':
                    this.resetView();
                    break;
                case 'l':
                    this.toggleLidar();
                    break;
                case 'b':
                    this.toggleRobot();
                    break;
                case 'escape':
                    this.unsubscribeAll();
                    break;
                case 'p':
                    document.getElementById('togglePanelBtn').click();
                    break;
            }
        });

        // 全屏变化监听
        document.addEventListener('fullscreenchange', () => {
            const fullscreenBtn = document.getElementById('fullscreenBtn');
            if (document.fullscreenElement) {
                fullscreenBtn.textContent = '⛶';
                this.logMessage("已进入全屏模式", "info");
            } else {
                fullscreenBtn.textContent = '⛶';
                this.logMessage("已退出全屏模式", "info");
            }
        });
    }
}

// 页面加载完成后初始化应用
window.addEventListener('DOMContentLoaded', () => {
    const app = new ROS2DMapViewer();
    window.ros2dApp = app; // 暴露到全局以便调试

    console.log('ROS2D 地图查看器已加载');
    console.log('可用快捷键:');
    console.log('  R - 重置视图');
    console.log('  L - 切换雷达显示');
    console.log('  B - 切换机器人显示');
    console.log('  P - 收起/展开控制面板');
    console.log('  ESC - 取消所有订阅');
});