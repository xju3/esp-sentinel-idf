// 全局变量初始化 - 用于WiFi配置匹配
// 确保这些变量在脚本执行前就存在
if (typeof window.wifiListLoaded === 'undefined') {
  window.wifiListLoaded = false;
}
if (typeof window.pendingWifiConfig === 'undefined') {
  window.pendingWifiConfig = null;
}
if (typeof window.savedWifiConfig === 'undefined') {
  window.savedWifiConfig = null;
}

// 滑块值显示函数 - 根据类型返回数值和单位
function formatSliderValue(minutes, type) {
  if (type === 'm') {
    return { number: minutes.toString(), unit: '分钟' };
  } else if (type === 'h') {
    const hours = minutes / 60;
    return { number: hours.toString(), unit: '小时' };
  } else {
    // 默认处理
    if (minutes < 60) {
      return { number: minutes.toString(), unit: '分钟' };
    } else {
      const hours = minutes / 60;
      return { number: hours.toString(), unit: '小时' };
    }
  }
}

/**
 * 通用刻度点生成函数
 * @param {number} min - 最小值
 * @param {number} max - 最大值
 * @param {number} tickInterval - 刻度间隔（生成所有刻度的间隔）
 * @param {number} labelInterval - 标签间隔（显示标签的值的间隔，0表示所有刻度都显示标签）
 * @param {string} type - 类型标识 ('s'=秒, 'm'=分钟, 'h'=小时)
 * @param {string} color - 刻度颜色
 * @returns {Array} 刻度点数组
 */
function generateScalePoints(min, max, tickInterval, labelInterval, type, color) {
  const points = [];
  
  // 特殊处理：总是包含最小值
  let firstValue = min;
  
  // 计算第一个点的分钟数
  let firstMinutes;
  if (type === 's') {
    firstMinutes = firstValue / 60;
  } else if (type === 'm') {
    firstMinutes = firstValue;
  } else if (type === 'h') {
    firstMinutes = firstValue * 60;
  } else {
    firstMinutes = firstValue;
  }
  
  // 添加第一个点（总是显示标签）
  points.push({
    minutes: firstMinutes,
    label: firstValue.toString(),
    number: firstValue.toString(),
    color: color,
    type: type
  });
  
  // 生成中间的点
  // 从第一个tickInterval的倍数开始，但确保不小于min+1
  let startValue = Math.ceil((min + 1) / tickInterval) * tickInterval;
  if (startValue <= min) {
    startValue = min + tickInterval;
  }
  
  for (let value = startValue; value < max; value += tickInterval) {
    // 计算分钟数（根据类型转换）
    let minutes;
    if (type === 's') {
      minutes = value / 60; // 秒转分钟
    } else if (type === 'm') {
      minutes = value; // 分钟
    } else if (type === 'h') {
      minutes = value * 60; // 小时转分钟
    } else {
      minutes = value; // 默认
    }
    
    // 决定是否显示标签
    let label = '';
    
    // 如果labelInterval为0，显示所有刻度的标签
    if (labelInterval === 0) {
      label = value.toString();
    }
    // 如果labelInterval>0，显示值是labelInterval倍数的点的标签
    else if (value % labelInterval === 0) {
      label = value.toString();
    }
    
    points.push({
      minutes: minutes,
      label: label,
      number: value.toString(),
      color: color,
      type: type
    });
  }
  
  // 总是包含最大值（总是显示标签）
  let maxMinutes;
  if (type === 's') {
    maxMinutes = max / 60;
  } else if (type === 'm') {
    maxMinutes = max;
  } else if (type === 'h') {
    maxMinutes = max * 60;
  } else {
    maxMinutes = max;
  }
  
  points.push({
    minutes: maxMinutes,
    label: max.toString(),
    number: max.toString(),
    color: color,
    type: type
  });
  
  return points;
}

// 根据时间单位生成关键时间点（使用通用函数）
function getSliderPoints(timeUnit) {
  // timeUnit: 1=秒, 2=分钟, 3=小时
  if (timeUnit === '1') { // 秒 - 刻度间隔为5，标签间隔为5（显示5的倍数）
    return generateScalePoints(1, 60, 5, 10, 's', '#f40404');
  } else if (timeUnit === '2') { // 分钟 - 刻度间隔为5，标签间隔为5（显示5的倍数）
    return generateScalePoints(1, 60, 5, 10, 'm', 'rgb(242, 2, 226)');
  } else { // 小时 - 刻度间隔为1，标签间隔为4（显示4的倍数）
    return generateScalePoints(1, 24, 1, 4, 'h', '#34b306');
  }
}

// 全局变量存储当前滑块点
let currentSliderPoints = getSliderPoints('3'); // 默认小时

// WiFi扫描相关常量
const MAX_WIFI_RETRIES = 3; // 最多重试3次
const WIFI_RETRY_INTERVAL = 2000; // 重试间隔2秒

// 生成滑块刻度线和数字标签
function generateSliderTicksAndNumbers(sliderId, ticksId, numbersId, timeUnit) {
  const ticksContainer = document.getElementById(ticksId);
  const numbersContainer = document.getElementById(numbersId);
  const slider = document.getElementById(sliderId);
  
  // 根据时间单位获取对应的滑块点
  const points = getSliderPoints(timeUnit);
  
  // 设置滑块属性
  slider.min = 0;
  slider.max = points.length - 1;
  slider.step = 1;
  
  // 清除现有内容
  ticksContainer.innerHTML = '';
  numbersContainer.innerHTML = '';
  
  // 生成刻度线 - 等间距分布
  points.forEach((point, index) => {
    const percent = (index / (points.length - 1)) * 100;
    
    // 创建刻度线
    const tick = document.createElement('div');
    tick.className = 'slider-tick';
    
    // 如果有标签，则为主要刻度（更高）
    if (point.label && point.label.trim() !== '') {
      tick.classList.add('major');
    }
    
    tick.style.left = `${percent}%`;
    
    // 创建刻度标签
    const tickLabel = document.createElement('div');
    tickLabel.className = 'slider-tick-label';
    tickLabel.textContent = point.label;
    tickLabel.style.left = `${percent}%`;
    // 设置标签颜色
    tickLabel.style.color = point.color;
    
    // 只有当标签不为空时才添加标签
    if (point.label && point.label.trim() !== '') {
      ticksContainer.appendChild(tickLabel);
    }
    ticksContainer.appendChild(tick);
  });
  
  // 确保两个滑块的刻度完全一致
  console.log(`Generated ticks for ${sliderId}: ${points.length} points (timeUnit: ${timeUnit})`);
}

// 确保两个滑块刻度一致
function ensureConsistentTicks() {
  const detectionTicks = document.getElementById('detection-ticks');
  const reportingTicks = document.getElementById('reporting-ticks');
  
  if (detectionTicks.children.length !== reportingTicks.children.length) {
    console.warn('Tick counts differ, regenerating...');
    // 获取当前时间单位
    const detectType = document.querySelector('input[name="detect-type"]:checked').value;
    const reportType = document.querySelector('input[name="report-type"]:checked').value;
    
    generateSliderTicksAndNumbers('detection-freq', 'detection-ticks', 'detection-numbers', detectType);
    generateSliderTicksAndNumbers('reporting-freq', 'reporting-ticks', 'reporting-numbers', reportType);
  }
}

// 更新滑块值和显示
function updateSlider(sliderId, valueId, timeUnit) {
  const slider = document.getElementById(sliderId);
  const valueDisplay = document.getElementById(valueId);
  
  // 根据时间单位获取对应的滑块点
  const points = getSliderPoints(timeUnit);
  
  // 更新显示函数
  function updateDisplay() {
    const index = parseInt(slider.value);
    const point = points[index];
    
    if (point) {
      // 根据类型显示单位和数值
      let displayValue = point.number; // 使用number而不是label
      let unit = '';
      
      if (point.type === 's') {
        unit = '秒';
      } else if (point.type === 'm') {
        unit = '分钟';
      } else if (point.type === 'h') {
        unit = '小时';
      }
      
      // 显示数值和单位
      valueDisplay.textContent = displayValue + ' ' + unit;
    }
  }
  
  // 初始显示
  updateDisplay();
  
  // 滑块输入事件
  slider.addEventListener('input', updateDisplay);
  
  // 滑块变化事件（鼠标释放）
  slider.addEventListener('change', updateDisplay);
}

// 将RSSI转换为信号强度描述
function rssiToSignalStrength(rssi) {
  if (rssi >= -50) return '极强';
  if (rssi >= -60) return '强';
  if (rssi >= -70) return '中';
  if (rssi >= -80) return '弱';
  return '极弱';
}

// 显示/隐藏遮罩
function showMask(message, details) {
  const mask = document.getElementById('processing-mask');
  const maskMessage = document.getElementById('mask-message');
  const maskDetails = document.getElementById('mask-details');
  
  maskMessage.textContent = message || '正在处理中...';
  maskDetails.textContent = details || '请稍候';
  mask.classList.add('active');
}

function hideMask() {
  const mask = document.getElementById('processing-mask');
  mask.classList.remove('active');
}

// 更新热点状态
function updateHotspotState(state, details, retryCount) {
  // 页面上的热点状态显示已被移除，此函数变为无操作
  // 保持函数调用兼容性，但不执行任何UI更新
  console.log(`热点状态更新: state=${state}, details=${details}, retryCount=${retryCount}`);
  // 不进行任何DOM操作，因为相关元素已被移除
}

// 加载WiFi列表（从服务器获取真实数据）
function loadWifiList() {
  console.log('开始加载WiFi列表...');
  const select = document.getElementById('wifi-select');
  select.innerHTML = '<option value="">正在扫描WiFi网络...</option>';
  
  // 重要：重置WiFi列表加载标志，等待新的列表加载
  window.wifiListLoaded = false;
  
  // 重要：如果之前有保存的WiFi配置，重新设置为待处理配置，以便刷新后能重新匹配
  if (window.savedWifiConfig) {
    console.log('刷新WiFi列表，重新设置待处理配置:', window.savedWifiConfig.ssid);
    window.pendingWifiConfig = {
      ssid: window.savedWifiConfig.ssid,
      password: window.savedWifiConfig.password
    };
  } else {
    console.log('刷新WiFi列表，没有保存的WiFi配置');
    window.pendingWifiConfig = null;
  }
  
  // 更新连接状态
  const statusDiv = document.getElementById('connection-status');
  statusDiv.textContent = '正在扫描WiFi网络...';
  statusDiv.style.background = '#fff3cd';
  statusDiv.style.borderColor = '#ffeaa7';
  statusDiv.style.color = '#856404';
  
  // 更新热点状态
  updateHotspotState('scanning', '正在扫描WiFi网络...', 0);
  
  // 显示遮罩 - 更详细的状态信息
  showMask('WiFi热点查询', '正在启动WiFi扫描，请稍候...');
  
  // 第一步：开始WiFi扫描
  fetch('/api/wifi-scan-start')
    .then(response => {
      if (!response.ok) {
        throw new Error('启动WiFi扫描失败');
      }
      return response.text();
    })
    .then(() => {
      console.log('WiFi扫描启动成功，2秒后获取列表');
      // 更新遮罩状态
      showMask('WiFi热点查询', '扫描已启动，正在搜索可用网络...');
      
      // 第二步：2秒后获取WiFi列表
      setTimeout(() => {
        fetchWifiListWithRetry();
      }, 2000);
    })
    .catch(error => {
      console.error('WiFi扫描启动失败:', error);
      select.innerHTML = '<option value="">扫描失败，请重试</option>';
      statusDiv.textContent = 'WiFi扫描失败';
      statusDiv.style.background = '#f8d7da';
      statusDiv.style.borderColor = '#f5c6cb';
      statusDiv.style.color = '#721c24';
      
      // 更新热点状态
      updateHotspotState('inactive', 'WiFi扫描失败', 0);
      
      // 隐藏遮罩
      hideMask();
      
      // 显示错误提示
      //alert('WiFi扫描启动失败: ' + error.message);
    });
}

// 获取WiFi列表并处理重试逻辑（最多3次重试，每次间隔2秒）
function fetchWifiListWithRetry(retryCount = 0) {
  const maxRetries = 3; // 最多重试3次
  const retryInterval = 2000; // 重试间隔2秒
  const select = document.getElementById('wifi-select');
  const statusDiv = document.getElementById('connection-status');
  
  console.log(`获取WiFi列表，尝试 ${retryCount + 1}/${maxRetries}`);
  
  // 更新热点状态
  updateHotspotState('scanning', `正在获取WiFi列表... (${retryCount + 1}/${maxRetries})`, retryCount);
  
  // 更新遮罩消息 - 更详细的状态信息
  showMask('WiFi热点查询', `正在获取WiFi列表 (尝试 ${retryCount + 1}/${maxRetries})...`);
  
  fetch('/api/wifi-list')
    .then(response => {
      if (!response.ok) {
        throw new Error('获取WiFi列表失败');
      }
      return response.json();
    })
    .then(data => {
      console.log('获取WiFi列表响应:', data);
      
      // 检查是否为 'processing' 状态
      let isProcessing = false;
      let networks = [];
      
      if (typeof data === 'object' && data !== null) {
        // 如果是对象，检查是否有 status 字段
        if (data.status === 'processing') {
          isProcessing = true;
        } else if (Array.isArray(data)) {
          // 如果是数组，直接作为网络列表
          networks = data;
        } else if (data.status && data.status !== 'processing') {
          // 其他状态，可能出错
          console.error('WiFi列表返回错误状态:', data);
          throw new Error(`WiFi扫描失败: ${data.status}`);
        }
      } else if (typeof data === 'string' && data === 'processing') {
        // 兼容旧格式（字符串）
        isProcessing = true;
      }

      if (isProcessing) {
        // 服务器还在处理，2秒后重试
        if (retryCount < maxRetries) {
          console.log(`服务器仍在处理中，${retryInterval/1000}秒后重试 (${retryCount + 1}/${maxRetries})`);
          statusDiv.textContent = `正在获取WiFi列表... (${retryCount + 1}/${maxRetries})`;
          
          // 更新遮罩状态
          showMask('WiFi热点查询', `服务器仍在处理中，${retryInterval/1000}秒后重试...`);
          
          setTimeout(() => {
            fetchWifiListWithRetry(retryCount + 1);
          }, retryInterval);
        } else {
          // 重试次数用完，显示无可用热点
          console.log('WiFi扫描超时或未找到网络');
          statusDiv.textContent = 'WiFi扫描超时，未找到可用网络';
          statusDiv.style.background = '#fff3cd';
          statusDiv.style.borderColor = '#ffeaa7';
          statusDiv.style.color = '#856404';
          
          displayWifiNetworks([]); // 显示空列表/无可用热点
          updateHotspotState('inactive', '未找到可用WiFi热点', maxRetries);
          hideMask();
        }
      } else {
        // 成功获取网络列表
        console.log(`成功获取WiFi列表，找到 ${networks.length} 个网络`);
        
        // 更新遮罩状态为处理完成
        showMask('WiFi热点查询', `成功找到 ${networks.length} 个WiFi网络，正在处理...`);
        
        // 短暂延迟后显示结果
        setTimeout(() => {
          displayWifiNetworks(networks);
          
          // 更新热点状态
          updateHotspotState('active', `找到 ${networks.length} 个WiFi网络`, 0);
          
          // 隐藏遮罩
          hideMask();
        }, 500);
      }
    })
    .catch(error => {
      console.error('获取WiFi列表失败:', error);
      
      if (retryCount < maxRetries) {
        // 重试
        console.log(`获取失败，1秒后重试 (${retryCount + 1}/${maxRetries})`);
        statusDiv.textContent = `获取失败，正在重试... (${retryCount + 1}/${maxRetries})`;
        statusDiv.style.background = '#fff3cd';
        statusDiv.style.borderColor = '#ffeaa7';
        statusDiv.style.color = '#856404';
        
        // 更新遮罩状态
        showMask('WiFi热点查询', `获取失败，正在重试 (${retryCount + 1}/${maxRetries})...`);
        
        setTimeout(() => {
          fetchWifiListWithRetry(retryCount + 1);
        }, 1000);
      } else {
        // 重试次数用完，显示错误
        console.log('重试次数用完，获取WiFi列表失败');
        select.innerHTML = '<option value="">获取WiFi列表失败</option>';
        statusDiv.textContent = '获取WiFi列表失败';
        statusDiv.style.background = '#f8d7da';
        statusDiv.style.borderColor = '#f5c6cb';
        statusDiv.style.color = '#721c24';
        
        // 更新热点状态
        updateHotspotState('inactive', '获取WiFi列表失败', maxRetries);
        
        // 隐藏遮罩
        hideMask();
        
        // 禁用提交按钮
        disableSubmitButton('WiFi列表获取失败，无法提交');
        
        // 显示错误提示
        //alert('获取WiFi列表失败: ' + error.message);
      }
    });
}

// 显示WiFi网络列表
function displayWifiNetworks(networks) {
  const select = document.getElementById('wifi-select');
  const statusDiv = document.getElementById('connection-status');
  
  // 清空现有选项
  select.innerHTML = '<option value="">请选择WiFi网络</option>';
  
  if (!networks || networks.length === 0) {
    select.innerHTML = '<option value="">未找到可用WiFi网络</option>';
    statusDiv.textContent = '未找到可用WiFi网络';
    statusDiv.style.background = '#f8f9fa';
    statusDiv.style.borderColor = '#e9ecef';
    statusDiv.style.color = '#6c757d';
    return;
  }
  
  // 过滤掉信号弱和极弱的网络（RSSI >= -70 表示信号强度为"中"、"强"或"极强"）
  const filteredNetworks = networks.filter(network => {
    const signalStrength = rssiToSignalStrength(network.rssi);
    return signalStrength !== '弱' && signalStrength !== '极弱';
  });
  
  if (filteredNetworks.length === 0) {
    select.innerHTML = '<option value="">未找到信号良好的WiFi网络</option>';
    statusDiv.textContent = '未找到信号良好的WiFi网络';
    statusDiv.style.background = '#fff3cd';
    statusDiv.style.borderColor = '#ffeaa7';
    statusDiv.style.color = '#856404';
    return;
  }
  
  // 按信号强度排序（RSSI值越大，信号越强）
  filteredNetworks.sort((a, b) => b.rssi - a.rssi);
  
  // 添加WiFi网络选项
  filteredNetworks.forEach(network => {
    const option = document.createElement('option');
    option.value = network.ssid;
    const signalStrength = rssiToSignalStrength(network.rssi);
    const encrypted = network.enc === 1 ? '加密' : '开放';
    option.textContent = `${network.ssid} (${signalStrength}, ${network.rssi} dBm, ${encrypted})`;
    option.dataset.encrypted = network.enc.toString();
    select.appendChild(option);
  });
  
  // 更新连接状态
  const originalCount = networks.length;
  const filteredCount = filteredNetworks.length;
  const removedCount = originalCount - filteredCount;
  
  let statusText = `找到 ${filteredCount} 个信号良好的WiFi网络`;
  if (removedCount > 0) {
    statusText += ` (已过滤 ${removedCount} 个信号弱的网络)`;
  }
  
  statusDiv.textContent = statusText;
  statusDiv.style.background = '#d4edda';
  statusDiv.style.borderColor = '#c3e6cb';
  statusDiv.style.color = '#155724';
  
  // WiFi列表已加载完成，触发配置匹配事件
  console.log('WiFi列表加载完成，触发配置匹配');
  window.wifiListLoaded = true;
  
  // 如果有待处理的WiFi配置，尝试匹配
  if (window.pendingWifiConfig) {
    trySelectWifiFromConfig();
  }
}

// 加载配置数据
function loadConfigData() {
  showMask('正在加载配置', '请稍候，正在获取设备配置...');
  
  fetch('/api/config')
    .then(response => {
      if (!response.ok) {
        throw new Error('获取配置失败');
      }
      return response.json();
    })
    .then(config => {
      // 填充表单数据
      fillFormWithConfig(config);
      
      // 隐藏遮罩
      hideMask();
      
      // 启用提交按钮
      enableSubmitButton();
      
      console.log('配置加载成功:', config);
    })
    .catch(error => {
      console.error('加载配置失败:', error);
      
      // 隐藏遮罩
      hideMask();
      
      // 禁用提交按钮并显示错误
      disableSubmitButton('配置加载失败，无法提交');
      
      // 显示错误信息
      //alert('加载配置失败: ' + error.message);
    });
}

// 用配置数据填充表单
function fillFormWithConfig(config) {
  // 设备编号
  if (config.deviceId) {
    document.getElementById('device-id').value = config.deviceId;
  }
  
  // 设备名称
  if (config.deviceName) {
    document.getElementById('device-name').value = config.deviceName;
  }
  
  // 机型
  if (config.modelType) {
    document.getElementById('model-type').value = config.modelType;
  }
  
  // 已用年限
  if (config.years !== undefined) {
    document.getElementById('years-used').value = config.years;
  }
  
  // HTTP服务器地址
  if (config.host && config.host.http) {
    document.getElementById('host-http').value = config.host.http;
  }
  
  // MQTT服务器地址
  if (config.host && config.host.mqtt) {
    document.getElementById('host-mqtt').value = config.host.mqtt;
  }
  
  // 检测频率 - 找到对应的滑块位置
  if (config.detect) {
    console.log('加载检测频率配置:', config.detect);
    // 设置检测频率的时间单位
    const detectType = config.detect.type.toString();
    console.log(`检测频率类型: ${detectType}, 值: ${config.detect.value}`);
    
    const detectRadio = document.querySelector(`input[name="detect-type"][value="${detectType}"]`);
    if (detectRadio) {
      console.log(`找到检测频率单选按钮，选中值: ${detectType}`);
      detectRadio.checked = true;
      // 触发change事件更新滑块刻度
      const event = new Event('change');
      detectRadio.dispatchEvent(event);
    } else {
      console.log(`未找到检测频率单选按钮，值: ${detectType}`);
    }
    
    // 找到对应的滑块位置
    const detectionIndex = findSliderIndexByMinutes(config.detect.value, detectType);
    console.log(`检测频率匹配结果: 索引=${detectionIndex}, 值=${config.detect.value}, 类型=${detectType}`);
    
    if (detectionIndex !== -1) {
      document.getElementById('detection-freq').value = detectionIndex;
      updateSlider('detection-freq', 'detection-value', detectType);
      console.log(`检测频率滑块已设置为索引: ${detectionIndex}`);
    } else {
      console.log(`检测频率未找到匹配的滑块位置，使用默认值`);
      document.getElementById('detection-freq').value = 0; // 默认第一项
      updateSlider('detection-freq', 'detection-value', detectType);
    }
  }
  
  // 上报频率 - 找到对应的滑块位置
  if (config.report) {
    // 设置上报频率的时间单位
    const reportType = config.report.type.toString();
    const reportRadio = document.querySelector(`input[name="report-type"][value="${reportType}"]`);
    if (reportRadio) {
      reportRadio.checked = true;
      // 触发change事件更新滑块刻度
      const event = new Event('change');
      reportRadio.dispatchEvent(event);
    }

    const reportingIndex = findSliderIndexByMinutes(config.report.value, reportType);
    if (reportingIndex !== -1) {
      document.getElementById('reporting-freq').value = reportingIndex;
      updateSlider('reporting-freq', 'reporting-value', reportType);
    }
  }
  
  // 通讯方式
  if (config.comm_type) {
    const commTypeRadio = document.querySelector(`input[name="comm-type"][value="${config.comm_type}"]`);
    if (commTypeRadio) {
      commTypeRadio.checked = true;
      // 触发change事件更新UI
      const event = new Event('change');
      commTypeRadio.dispatchEvent(event);
    }
  }

  // WiFi SSID - 设置WiFi选择框（延迟到WiFi列表加载完成后执行）
  // 注意：这个匹配操作将在WiFi列表完全加载后由事件触发
  if (config.wifi && config.wifi.ssid) {
    // 验证SSID是否为有效字符串（不是占位符数字）
    const configSsid = String(config.wifi.ssid).trim();
    
    // 检查是否为有效SSID：非空字符串，且不是数字占位符（如"1"）
    if (configSsid === '' || configSsid === '1') {
      console.log(`配置中的WiFi SSID无效或为占位符: "${configSsid}"，跳过自动选择`);
      return;
    }
    
    console.log(`配置中有WiFi SSID: "${configSsid}"，已保存配置并等待WiFi列表加载完成后尝试匹配`);
    
    // 保存配置SSID到永久存储，供WiFi列表加载完成事件使用（包括刷新后）
    window.savedWifiConfig = {
      ssid: configSsid,
      password: config.wifi.pass
    };
    
    // 同时设置pendingWifiConfig用于当前匹配
    window.pendingWifiConfig = {
      ssid: configSsid,
      password: config.wifi.pass
    };
    
    // 如果WiFi列表已经加载，立即尝试匹配
    if (window.wifiListLoaded) {
      trySelectWifiFromConfig();
    }
  }
}

// 根据分钟数找到对应的滑块索引
function findSliderIndexByMinutes(minutes, timeUnit) {
  const points = getSliderPoints(timeUnit);

  // 这里的 minutes 实际上是“用户配置值”，语义与 type 一致：
  // type=1 秒，type=2 分钟，type=3 小时。
  // slider points 的 point.number 也是该单位下的数字，因此直接用 number 匹配。
  console.log(`findSliderIndexByMinutes: 寻找value=${minutes}, timeUnit=${timeUnit}, 共有${points.length}个点`);

  const searchValue = parseFloat(minutes);
  if (Number.isNaN(searchValue)) return 0;
  
  // 首先尝试精确匹配
  for (let i = 0; i < points.length; i++) {
    const point = points[i];
    const n = parseFloat(point.number);
    if (!Number.isNaN(n) && n === searchValue) {
      return i;
    }
  }
  
  // 如果没有精确匹配，尝试寻找最接近的值（对于边界情况）
  console.log(`  未找到value=${searchValue}的精确匹配，尝试寻找最接近的值`);
  
  let closestIndex = 0;
  let minDiff = Math.abs(parseFloat(points[0].number) - searchValue);
  
  for (let i = 1; i < points.length; i++) {
    const diff = Math.abs(parseFloat(points[i].number) - searchValue);
    if (diff < minDiff) {
      minDiff = diff;
      closestIndex = i;
    }
  }

  console.log(`  找到最接近的值: value=${points[closestIndex].number} (索引${closestIndex}, 差值${minDiff})`);
  return closestIndex;
}

// 启用提交按钮
function enableSubmitButton() {
  const submitBtn = document.querySelector('.submit-btn');
  submitBtn.disabled = false;
  submitBtn.style.opacity = '1';
  submitBtn.style.cursor = 'pointer';
  submitBtn.title = '';
}

// 禁用提交按钮并显示错误信息
function disableSubmitButton(errorMessage) {
  const submitBtn = document.querySelector('.submit-btn');
  submitBtn.disabled = true;
  submitBtn.style.opacity = '0.5';
  submitBtn.style.cursor = 'not-allowed';
  submitBtn.title = errorMessage;
}

// WiFi选择变化时显示密码输入框
document.getElementById('wifi-select').addEventListener('change', function() {
  const selectedOption = this.options[this.selectedIndex];
  const isEncrypted = selectedOption.dataset.encrypted === '1';
  const passwordContainer = document.getElementById('wifi-password-container');
  
  if (isEncrypted && selectedOption.value) {
    passwordContainer.classList.add('show');
    document.getElementById('wifi-password').required = true;
  } else {
    passwordContainer.classList.remove('show');
    document.getElementById('wifi-password').required = false;
  }
});

// 通讯方式切换监听
function setupCommTypeChangeListener() {
  console.log('setupCommTypeChangeListener: 开始设置通讯方式监听');
  const commTypeRadios = document.querySelectorAll('input[name="comm-type"]');
  const wifiConfigContainer = document.getElementById('wifi-config-container');
  
  if (!commTypeRadios.length) {
    console.error('setupCommTypeChangeListener: 找不到通讯方式单选按钮');
    return;
  }
  
  if (!wifiConfigContainer) {
    console.error('setupCommTypeChangeListener: 找不到WiFi配置容器元素 #wifi-config-container');
    return;
  }
  console.log('setupCommTypeChangeListener: 找到必要元素，绑定事件监听');
  // 为每个单选按钮直接绑定事件监听器
  commTypeRadios.forEach(radio => {
    radio.addEventListener('change', function(event) {
      const target = event.target;
      console.log('通讯方式 change event:', target.tagName, target.name, target.value, target.checked);

      // 确保事件来自 name="comm-type" 的 radio 按钮
      if (target && target.name === 'comm-type') {
        console.log('通讯方式切换事件触发，选择的值:', target.value);
        
        if (target.value === '2') { // WiFi
          console.log('切换到WiFi，显示WiFi配置容器');
          wifiConfigContainer.classList.add('show');
          // 总是重新加载WiFi列表，确保获取最新的可用网络
          loadWifiList();
        } else { // 4G
          console.log('切换到其他方式（如4G），隐藏WiFi配置容器');
          wifiConfigContainer.classList.remove('show');
        }
      }
    });
  });
  
  // 初始检查：如果WiFi被选中，确保WiFi配置容器显示
  // 这里再次确认当前选中的值，因为之前的初始化可能已经被覆盖或尚未生效
  const selectedCommType = document.querySelector('input[name="comm-type"]:checked');
  console.log('setupCommTypeChangeListener: 初始检查选中状态:', selectedCommType ? selectedCommType.value : '无');

  if (selectedCommType && selectedCommType.value === '2') {
    console.log('初始化时WiFi被选中，显示WiFi配置容器');
    wifiConfigContainer.classList.add('show');
  } else {
    // 确保默认隐藏
    wifiConfigContainer.classList.remove('show');
  }
}

// 刷新WiFi列表按钮
document.getElementById('refresh-wifi').addEventListener('click', loadWifiList);

// 表单提交
document.getElementById('device-form').addEventListener('submit', function(event) {
  event.preventDefault();
  
  // 显示遮罩
  showMask('正在提交配置', '请稍候，正在保存设备配置...');
  
  // 获取时间单位
  const detectType = document.querySelector('input[name="detect-type"]:checked').value;
  const reportType = document.querySelector('input[name="report-type"]:checked').value;
  
  // 获取对应的滑块点
  const detectPoints = getSliderPoints(detectType);
  const reportPoints = getSliderPoints(reportType);

  // 将滑块点转换为“用户配置值”：
  // value 与 type 语义一致：type=1 秒，type=2 分钟，type=3 小时
  function sliderPointToUserValue(point) {
    if (!point) return 1;
    const n = parseFloat(point.number);
    if (Number.isNaN(n) || n <= 0) return 1;
    return Math.round(n);
  }
  
  // 收集表单数据 - 与 default_config.json 结构对齐
  const formData = {
    deviceId: document.getElementById('device-id').value,
    deviceName: document.getElementById('device-name').value,
    modelType: document.getElementById('model-type').value,
    years: parseFloat(document.getElementById('years-used').value) || 0,
    host: {
      http: document.getElementById('host-http').value,
      mqtt: document.getElementById('host-mqtt').value
    },
    detect: {
      type: parseInt(detectType),
      value: sliderPointToUserValue(detectPoints[parseInt(document.getElementById('detection-freq').value)])
    },
    report: {
      type: parseInt(reportType),
      value: sliderPointToUserValue(reportPoints[parseInt(document.getElementById('reporting-freq').value)])
    },
    comm_type: parseInt(document.querySelector('input[name="comm-type"]:checked').value),
    wifi: {
      ssid: document.getElementById('wifi-select').value,
      pass: document.getElementById('wifi-password').value
    },
    configured: true
  };
  
  // 验证必填字段
  if (!formData.deviceId || !formData.deviceName || !formData.modelType || 
      !formData.host.http || !formData.host.mqtt) {
    alert('请填写所有必填字段（标有*的字段）');
    hideMask();
    return;
  }
  
  // 如果选择了加密WiFi但未输入密码
  const selectedOption = document.getElementById('wifi-select').options[document.getElementById('wifi-select').selectedIndex];
  if (selectedOption.dataset.encrypted === '1' && selectedOption.value && !formData.wifi.pass) {
    alert('请为加密WiFi输入密码');
    hideMask();
    return;
  }
  
  // 显示提交状态
  const submitBtn = document.querySelector('.submit-btn');
  const originalText = submitBtn.textContent;
  submitBtn.textContent = '提交中...';
  submitBtn.disabled = true;
  
  // 调用 /api/save 接口保存配置
  fetch('/api/save', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(formData)
  })
  .then(response => {
    if (!response.ok) {
      throw new Error('保存配置失败');
    }
    return response.json();
  })
  .then(result => {
    // 隐藏遮罩
    hideMask();
    
    const detectionPoint = detectPoints[parseInt(document.getElementById('detection-freq').value)];
    const reportingPoint = reportPoints[parseInt(document.getElementById('reporting-freq').value)];
    
    const detectionText = detectionPoint.label + (detectionPoint.type === 's' ? '秒' : detectionPoint.type === 'm' ? '分钟' : '小时');
    const reportingText = reportingPoint.label + (reportingPoint.type === 's' ? '秒' : reportingPoint.type === 'm' ? '分钟' : '小时');
    
    console.log('保存成功:', result);
    
    //alert('配置保存成功！\n设备编号: ' + formData.deviceId + '\n检测频率: ' + detectionText + '\n上报频率: ' + reportingText);
    
    submitBtn.textContent = originalText;
    submitBtn.disabled = false;
    
    // 更新连接状态
    document.getElementById('connection-status').textContent = '配置已保存，等待连接...';
    document.getElementById('connection-status').style.background = '#e8f5e9';
    document.getElementById('connection-status').style.borderColor = '#c8e6c9';
    document.getElementById('connection-status').style.color = '#2e7d32';
  })
  .catch(error => {
    console.error('保存配置失败:', error);
    
    // 隐藏遮罩
    hideMask();
    
    alert('保存配置失败: ' + error.message);
    
    submitBtn.textContent = originalText;
    submitBtn.disabled = false;
  });
});

// 窗口大小改变时重新生成刻度
let resizeTimeout;
function handleResize() {
  clearTimeout(resizeTimeout);
  resizeTimeout = setTimeout(function() {
    // 获取当前时间单位
    const detectType = document.querySelector('input[name="detect-type"]:checked').value;
    const reportType = document.querySelector('input[name="report-type"]:checked').value;
    
    // 重新生成检测频率滑块的刻度和数字
    generateSliderTicksAndNumbers('detection-freq', 'detection-ticks', 'detection-numbers', detectType);
    // 重新生成上报频率滑块的刻度和数字
    generateSliderTicksAndNumbers('reporting-freq', 'reporting-ticks', 'reporting-numbers', reportType);
    
    // 确保两个滑块刻度一致
    ensureConsistentTicks();
    
    console.log('Regenerated ticks on resize for mobile optimization');
  }, 250);
}

// 时间单位选择变化时重置滑块值
function setupTimeUnitChangeListeners() {
  // 检测频率时间单位变化
  const detectTypeRadios = document.querySelectorAll('input[name="detect-type"]');
  detectTypeRadios.forEach(radio => {
    radio.addEventListener('change', function() {
      if (this.checked) {
        // 重新生成检测频率滑块的刻度
        generateSliderTicksAndNumbers('detection-freq', 'detection-ticks', 'detection-numbers', this.value);
        // 重置检测频率滑块值为1（最小值）
        document.getElementById('detection-freq').value = 0; // 0对应1分钟/秒
        updateSlider('detection-freq', 'detection-value', this.value);
        console.log('检测频率时间单位改为:', this.value, '，滑块重置为1，刻度已更新');
      }
    });
  });
  
  // 上报频率时间单位变化
  const reportTypeRadios = document.querySelectorAll('input[name="report-type"]');
  reportTypeRadios.forEach(radio => {
    radio.addEventListener('change', function() {
      if (this.checked) {
        // 重新生成上报频率滑块的刻度
        generateSliderTicksAndNumbers('reporting-freq', 'reporting-ticks', 'reporting-numbers', this.value);
        // 重置上报频率滑块值为1（最小值）
        document.getElementById('reporting-freq').value = 0; // 0对应1分钟/秒
        updateSlider('reporting-freq', 'reporting-value', this.value);
        console.log('上报频率时间单位改为:', this.value, '，滑块重置为1，刻度已更新');
      }
    });
  });
}

// 初始化滑块
function initializeApp() {
  // 获取初始时间单位
  const detectType = document.querySelector('input[name="detect-type"]:checked').value;
  const reportType = document.querySelector('input[name="report-type"]:checked').value;
  
  // 生成检测频率滑块的刻度和数字
  generateSliderTicksAndNumbers('detection-freq', 'detection-ticks', 'detection-numbers', detectType);
  // 生成上报频率滑块的刻度和数字
  generateSliderTicksAndNumbers('reporting-freq', 'reporting-ticks', 'reporting-numbers', reportType);
  
  // 确保两个滑块刻度一致
  ensureConsistentTicks();
  
  // 初始化滑块值显示
  updateSlider('detection-freq', 'detection-value', detectType);
  updateSlider('reporting-freq', 'reporting-value', reportType);
  
  // 设置时间单位变化监听
  setupTimeUnitChangeListeners();
  
  setupCommTypeChangeListener();
  
  // 加载初始化页面
  loadConfigData();
  
  // 检查当前选择的通讯方式，如果是WiFi则加载WiFi列表
  const selectedCommType = document.querySelector('input[name="comm-type"]:checked').value;
  if (selectedCommType === '2') { // WiFi
    loadWifiList();
  }
  
  // 添加窗口大小改变监听
  window.addEventListener('resize', handleResize);
}

// 从配置中尝试选择WiFi
function trySelectWifiFromConfig() {
  if (!window.pendingWifiConfig) {
    console.log('没有待处理的WiFi配置');
    return;
  }
  
  const { ssid, password } = window.pendingWifiConfig;
  const wifiSelect = document.getElementById('wifi-select');
  
  console.log(`尝试从配置中选择WiFi SSID: "${ssid}"`);
  
  // 查找匹配的选项
  let found = false;
  for (let i = 0; i < wifiSelect.options.length; i++) {
    const option = wifiSelect.options[i];
    if (option.value === ssid) {
      wifiSelect.selectedIndex = i;
      console.log(`成功匹配WiFi SSID: "${ssid}"，已选中第${i}个选项`);
      
      // 触发change事件以显示密码输入框
      const event = new Event('change');
      wifiSelect.dispatchEvent(event);
      
      // 如果配置中有密码，填充密码字段
      if (password) {
        document.getElementById('wifi-password').value = password;
        console.log(`已填充WiFi密码（长度: ${password.length}）`);
      }
      found = true;
      break;
    }
  }
  
  if (!found) {
    console.log(`配置的WiFi SSID "${ssid}" 不在当前WiFi列表中`);
  }
  
  // 清除待处理配置，避免重复匹配
  window.pendingWifiConfig = null;
}

// 等待多语言初始化完成后再初始化应用
document.addEventListener('DOMContentLoaded', function() {
  // 等待一小段时间确保多语言脚本已加载
  setTimeout(initializeApp, 100);
});
