// 多语言支持
const locales = {
  'zh-CN': {
    title: 'Sentinel 设备配置',
    subtitle: '请填写设备信息并设置监测参数',
    hotspotStatus: '执行热点状态',
    wifiHotspotStatus: 'WiFi 热点状态',
    status: '状态',
    details: '详情',
    retryCount: '重试次数',
    basicInfo: '设备信息',
    deviceId: '设备编号 *',
    deviceIdPlaceholder: '例如: SN-2025-001',
    deviceName: '设备名称 *',
    deviceNamePlaceholder: '例如: 主电机监测器',
    modelType: '机型 *',
    selectModel: '请选择机型',
    modelA: 'A型 - 标准工业型',
    modelB: 'B型 - 高温防爆型',
    modelC: 'C型 - 精密测量型',
    modelD: 'D型 - 移动便携型',
    yearsUsed: '已用年限 (年)',
    yearsUsedPlaceholder: '例如: 3.5',
    frequencySettings: '监测频率设置',
    detectFrequencyUnit: '检测频率时间单位',
    secondsOption: '秒',
    minutesOption: '分钟',
    hoursOption: '小时',
    detectFrequency: '检测频率',
    reportFrequencyUnit: '上报频率时间单位',
    reportFrequency: '上报频率',
    serverConfig: '服务器配置',
    serverAddress: '服务器地址 *',
    serverPlaceholder: '例如: http://example.com/api 或 mqtt://example.com:1883',
    serverNote: '支持 HTTP 和 MQTT 协议',
    communicationMethod: '通讯方式',
    comm4G: '4G',
    commWiFi: 'WiFi',
    wifiNetwork: 'WiFi 网络',
    scanningNetworks: '正在扫描可用网络...',
    refreshNetworks: '刷新网络列表',
    wifiPassword: 'WiFi 密码 *',
    wifiPasswordPlaceholder: '请输入WiFi密码',
    connectionStatus: '连接状态',
    notConnected: '未连接',
    submitConfig: '提交配置',
    processing: '正在处理中...',
    pleaseWait: '请稍候',
    scanningWiFi: '正在扫描WiFi网络...',
    searchingNetworks: '请稍候，正在搜索可用网络...',
    loadingConfig: '正在加载配置',
    gettingConfig: '请稍候，正在获取设备配置...',
    submittingConfig: '正在提交配置',
    savingConfig: '请稍候，正在保存设备配置...',
    configSaved: '配置已保存，等待连接...',
    hotspotState: {
      inactive: '未激活',
      scanning: '扫描中',
      active: '激活'
    },
    hotspotDetails: {
      waiting: '等待扫描WiFi网络',
      scanning: '正在扫描WiFi网络...',
      gettingList: '正在获取WiFi列表...',
      foundNetworks: '找到 {count} 个WiFi网络'
    }
  },
  'en-US': {
    title: 'Sentinel Device Configuration',
    subtitle: 'Please fill in device information and set monitoring parameters',
    hotspotStatus: 'Hotspot Status',
    wifiHotspotStatus: 'WiFi Hotspot Status',
    status: 'Status',
    details: 'Details',
    retryCount: 'Retry Count',
    basicInfo: 'Device Information',
    deviceId: 'Device ID *',
    deviceIdPlaceholder: 'e.g., SN-2025-001',
    deviceName: 'Device Name *',
    deviceNamePlaceholder: 'e.g., Main Motor Monitor',
    modelType: 'Model Type *',
    selectModel: 'Please select model',
    modelA: 'Type A - Standard Industrial',
    modelB: 'Type B - High Temperature Explosion-proof',
    modelC: 'Type C - Precision Measurement',
    modelD: 'Type D - Mobile Portable',
    yearsUsed: 'Years Used (years)',
    yearsUsedPlaceholder: 'e.g., 3.5',
    frequencySettings: 'Frequency Settings',
    detectFrequencyUnit: 'Detection Frequency Time Unit',
    secondsOption: 'Seconds',
    minutesOption: 'Minutes',
    hoursOption: 'Hours',
    detectFrequency: 'Detection Frequency',
    reportFrequencyUnit: 'Reporting Frequency Time Unit',
    reportFrequency: 'Reporting Frequency',
    serverConfig: 'Server Configuration',
    serverAddress: 'Server Address *',
    serverPlaceholder: 'e.g., http://example.com/api or mqtt://example.com:1883',
    serverNote: 'Supports HTTP and MQTT protocols',
    communicationMethod: 'Communication Method',
    comm4G: '4G',
    commWiFi: 'WiFi',
    wifiNetwork: 'WiFi Network',
    scanningNetworks: 'Scanning available networks...',
    refreshNetworks: 'Refresh Network List',
    wifiPassword: 'WiFi Password *',
    wifiPasswordPlaceholder: 'Please enter WiFi password',
    connectionStatus: 'Connection Status',
    notConnected: 'Not Connected',
    submitConfig: 'Submit Configuration',
    processing: 'Processing...',
    pleaseWait: 'Please wait',
    scanningWiFi: 'Scanning WiFi networks...',
    searchingNetworks: 'Please wait, searching for available networks...',
    loadingConfig: 'Loading configuration',
    gettingConfig: 'Please wait, getting device configuration...',
    submittingConfig: 'Submitting configuration',
    savingConfig: 'Please wait, saving device configuration...',
    configSaved: 'Configuration saved, waiting for connection...',
    hotspotState: {
      inactive: 'Inactive',
      scanning: 'Scanning',
      active: 'Active'
    },
    hotspotDetails: {
      waiting: 'Waiting to scan WiFi networks',
      scanning: 'Scanning WiFi networks...',
      gettingList: 'Getting WiFi list...',
      foundNetworks: 'Found {count} WiFi networks'
    }
  }
};

// 获取用户浏览器语言
function getUserLanguage() {
  const browserLang = navigator.language || navigator.userLanguage;
  // 检查是否支持该语言，如果不支持则使用默认语言
  if (locales[browserLang]) {
    return browserLang;
  }
  
  // 检查语言前缀（如 en-US 检查 en）
  const langPrefix = browserLang.split('-')[0];
  for (const lang in locales) {
    if (lang.startsWith(langPrefix)) {
      return lang;
    }
  }
  
  // 默认使用中文
  return 'zh-CN';
}

// 设置页面语言
function setPageLanguage(lang = null) {
  // 验证语言是否支持，如果不支持则自动检测
  let language;
  if (lang && locales[lang]) {
    language = lang;
  } else {
    language = getUserLanguage();
  }
  
  // 确保有对应的语言包
  const locale = locales[language] || locales['zh-CN'];
  
  // 更新页面标题
  document.title = locale.title;
  
  // 更新页面元素 - data-i18n 属性
  const elements = document.querySelectorAll('[data-i18n]');
  elements.forEach(element => {
    const key = element.getAttribute('data-i18n');
    const value = getNestedValue(locale, key);
    if (value !== undefined) {
      if (element.tagName === 'INPUT' || element.tagName === 'TEXTAREA') {
        // 对于输入框，我们只更新文本内容，placeholder 由 data-i18n-placeholder 处理
        if (element.type !== 'text' && element.type !== 'password' && element.type !== 'number') {
          element.textContent = value;
        }
      } else if (element.tagName === 'OPTION') {
        element.textContent = value;
      } else {
        element.textContent = value;
      }
    }
  });
  
  // 更新 placeholder 属性 - data-i18n-placeholder 属性
  const placeholderElements = document.querySelectorAll('[data-i18n-placeholder]');
  placeholderElements.forEach(element => {
    const key = element.getAttribute('data-i18n-placeholder');
    const value = getNestedValue(locale, key);
    if (value !== undefined) {
      element.placeholder = value;
    }
  });
  
  // 更新特定元素
  const titleElement = document.querySelector('h1');
  if (titleElement) titleElement.textContent = locale.title;
  
  const subtitleElement = document.querySelector('.subtitle');
  if (subtitleElement) subtitleElement.textContent = locale.subtitle;
  
  // 更新按钮文本
  const submitBtn = document.querySelector('.submit-btn');
  if (submitBtn) submitBtn.textContent = locale.submitConfig;
  
  const refreshBtn = document.getElementById('refresh-wifi');
  if (refreshBtn) refreshBtn.textContent = locale.refreshNetworks;
  
  // 更新选项文本
  const modelSelect = document.getElementById('model-type');
  if (modelSelect) {
    const options = modelSelect.options;
    if (options.length > 0) options[0].textContent = locale.selectModel;
    if (options.length > 1) options[1].textContent = locale.modelA;
    if (options.length > 2) options[2].textContent = locale.modelB;
    if (options.length > 3) options[3].textContent = locale.modelC;
    if (options.length > 4) options[4].textContent = locale.modelD;
  }
  
  // 更新单选按钮标签
  const detectLabels = document.querySelectorAll('[for^="detect-type-"]');
  if (detectLabels.length >= 1) detectLabels[0].textContent = locale.secondsOption;
  if (detectLabels.length >= 2) detectLabels[1].textContent = locale.minutesOption;
  if (detectLabels.length >= 3) detectLabels[2].textContent = locale.hoursOption;
  
  const reportLabels = document.querySelectorAll('[for^="report-type-"]');
  if (reportLabels.length >= 1) reportLabels[0].textContent = locale.secondsOption;
  if (reportLabels.length >= 2) reportLabels[1].textContent = locale.minutesOption;
  if (reportLabels.length >= 3) reportLabels[2].textContent = locale.hoursOption;
  
  // 更新通讯方式标签
  const commLabels = document.querySelectorAll('[for^="comm-type-"]');
  if (commLabels.length >= 1) commLabels[0].textContent = locale.comm4G;
  if (commLabels.length >= 2) commLabels[1].textContent = locale.commWiFi;
  
  // 保存语言设置到本地存储
  localStorage.setItem('preferredLanguage', language);
  
  // 设置html lang属性
  document.documentElement.lang = language;
}

// 获取嵌套对象的值
function getNestedValue(obj, path) {
  return path.split('.').reduce((current, key) => {
    return current && current[key] !== undefined ? current[key] : undefined;
  }, obj);
}

// 初始化多语言支持
document.addEventListener('DOMContentLoaded', function() {
  // 检查是否有保存的语言偏好
  const savedLang = localStorage.getItem('preferredLanguage');
  // 使用保存的语言或自动检测浏览器语言
  setPageLanguage(savedLang);
});

// 导出函数供其他文件使用
window.setPageLanguage = setPageLanguage;
window.getUserLanguage = getUserLanguage;
