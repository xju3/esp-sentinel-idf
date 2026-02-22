import 'katex/dist/katex.min.css';
import katex from 'katex';
import './style.css';

document.addEventListener('DOMContentLoaded', async () => {
  // 0. 加载配置数据
  async function loadConfigData() {
    try {
      const response = await fetch('/api/config');
      if (!response.ok) {
        console.warn('Failed to load config:', response.status);
        return null;
      }
      const config = await response.json();
      return config;
    } catch (error) {
      console.warn('Error loading config:', error);
      return null;
    }
  }

  // 0.1 填充表单数据
  function populateFormData(config) {
    if (!config) return;

    // ISO 标准
    if (config.iso?.standard) {
      const isoBtn = document.querySelector(`[data-value="${config.iso.standard}"]`);
      if (isoBtn) {
        isoBtn.click();
      }
    }

    // 机械类别 - 需要等待下拉菜单初始化
    if (config.iso?.category) {
      const categoryInput = document.getElementById('iso-category');
      const categoryLabel = document.getElementById('iso-category-label');
      if (categoryInput) {
        categoryInput.value = config.iso.category;
        
        // 查找对应的选项标签并更新显示
        const dropdown = document.getElementById('iso-category-dropdown');
        if (dropdown) {
          const item = dropdown.querySelector(`[data-value="${config.iso.category}"]`);
          if (item) {
            const label = item.querySelector('.font-medium')?.textContent || '未选择';
            if (categoryLabel) categoryLabel.textContent = label;
          }
        }
      }
    }

    // 安装基础
    if (config.iso?.foundation) {
      const foundationBtn = document.querySelector(`#foundation-select [data-value="${config.iso.foundation}"]`);
      if (foundationBtn) {
        foundationBtn.click();
      }
    }

    // 设备信息
    if (config.deviceId !== undefined) {
      const deviceIdInput = document.getElementById('device-id');
      if (deviceIdInput) deviceIdInput.value = config.deviceId || '';
    }

    if (config.deviceName !== undefined) {
      const deviceNameInput = document.getElementById('device-name');
      if (deviceNameInput) deviceNameInput.value = config.deviceName || '';
    }

    if (config.rpm !== undefined) {
      const rpmInput = document.getElementById('device-rpm');
      if (rpmInput) rpmInput.value = config.rpm || '';
    }

    if (config.years !== undefined) {
      const yearsInput = document.getElementById('years-used');
      if (yearsInput) yearsInput.value = config.years || '';
    }

    // 检测频率
    if (config.detect_interval !== undefined) {
      const freqBtn = document.querySelector(`#detect-frequency [data-value="${config.detect_interval}"]`);
      if (freqBtn) {
        freqBtn.click();
      }
    }

    // 上报周期
    if (config.report_cycle !== undefined) {
      const rangeInput = document.getElementById('report-cycle');
      if (rangeInput) {
        rangeInput.value = config.report_cycle;
        const cycleVal = document.getElementById('cycle-val');
        if (cycleVal) cycleVal.textContent = config.report_cycle;
      }
    }

    // 通讯方式
    if (config.comm_type !== undefined) {
      const commBtn = document.querySelector(`#comm-type [data-value="${config.comm_type}"]`);
      if (commBtn) {
        commBtn.click();
        
        // 如果是WiFi，需要加载并选择之前的SSID
        if (config.comm_type === 2 && config.wifi?.ssid) {
          // 延迟以允许WiFi选择框初始化
          setTimeout(() => {
            scanWifiNetworks().then(() => {
              const wifiSelect = document.getElementById('wifi-select');
              if (wifiSelect) {
                wifiSelect.value = config.wifi.ssid;
                
                // 触发change事件以更新密码框显示
                wifiSelect.dispatchEvent(new Event('change'));
                
                // 如果有密码，填充到密码框
                if (config.wifi.pass) {
                  const wifiPassword = document.getElementById('wifi-password');
                  if (wifiPassword) wifiPassword.value = config.wifi.pass;
                }
              }
            });
          }, 100);
        }
      }
    }
  }

  // 加载配置数据
  const configData = await loadConfigData();

  // 1. KaTeX 渲染示例 (在 ISO 标准描述中渲染公式)
  const katexContainer = document.getElementById('katex-formula');
  if (katexContainer) {
    katex.render("v_{RMS} = \\sqrt{\\frac{1}{T} \\int_{0}^{T} v^2(t) dt}", katexContainer, {
      throwOnError: false,
      displayMode: false
    });
  }

  // 2. 状态管理
  let currentStep = 1;
  const totalSteps = 5;

  // 3. UI 元素引用
  const panels = document.querySelectorAll('.panel');
  const steps = document.querySelectorAll('.step');
  const mask = document.getElementById('processing-mask');
  
  // 4. 导航逻辑
  function updateUI(stepIndex) {
    // 更新面板
    panels.forEach(p => p.classList.remove('active'));
    const targetPanel = document.getElementById(`panel-${stepIndex}`) || document.getElementById('panel-5');
    if(targetPanel) targetPanel.classList.add('active');

    // 更新步骤条
    steps.forEach(s => {
      const sIndex = parseInt(s.dataset.step);
      if (sIndex <= stepIndex) s.classList.add('active');
      else s.classList.remove('active');
    });

    currentStep = stepIndex;

    // 如果是预览页，收集并展示配置信息
    if (stepIndex === 5) {
      showConfigPreview();
    }
  }

  // 4.1 收集配置信息并展示预览
  function showConfigPreview() {
    // ISO 标准部分
    const isoStandardBtn = document.querySelector('#iso-standard .pill.active');
    const isoStandard = isoStandardBtn ? isoStandardBtn.textContent : 'ISO 10816 (通用)';
    document.getElementById('preview-iso-standard').textContent = isoStandard;

    const isoCategoryLabel = document.getElementById('iso-category-label');
    const isoCategoryText = isoCategoryLabel && isoCategoryLabel.textContent 
      ? isoCategoryLabel.textContent 
      : '未选择';
    document.getElementById('preview-iso-category').textContent = isoCategoryText;

    // 显示/隐藏基础选择预览
    const foundationGroup = document.getElementById('foundation-group');
    const foundationContainer = document.getElementById('preview-foundation-container');
    if (foundationGroup && foundationGroup.style.display !== 'none') {
      foundationContainer.style.display = 'block';
      const foundationBtn = document.querySelector('#foundation-select .pill.active');
      const foundationText = foundationBtn ? foundationBtn.textContent : '未选择';
      document.getElementById('preview-foundation').textContent = foundationText;
    } else {
      foundationContainer.style.display = 'none';
    }

    // 设备信息部分
    const deviceId = document.getElementById('device-id').value || '-';
    const deviceName = document.getElementById('device-name').value || '-';
    const deviceRpm = document.getElementById('device-rpm').value || '-';
    const yearsUsed = document.getElementById('years-used').value || '-';

    document.getElementById('preview-device-id').textContent = deviceId;
    document.getElementById('preview-device-name').textContent = deviceName;
    document.getElementById('preview-device-rpm').textContent = deviceRpm;
    document.getElementById('preview-years-used').textContent = yearsUsed;

    // 检测策略部分
    const detectFreqBtn = document.querySelector('#detect-frequency .pill.active');
    const detectFreq = detectFreqBtn ? detectFreqBtn.textContent : '30分钟';
    document.getElementById('preview-detect-frequency').textContent = detectFreq;

    const reportCycle = document.getElementById('report-cycle').value;
    document.getElementById('preview-report-cycle').textContent = `${reportCycle} 次检测后`;

    // 通讯配置部分
    const commTypeBtn = document.querySelector('#comm-type .pill.active');
    const commType = commTypeBtn ? commTypeBtn.textContent : '4G (LTE)';
    document.getElementById('preview-comm-type').textContent = commType;

    const wifiContainer = document.getElementById('preview-wifi-container');
    if (commType === '4G (LTE)') {
      wifiContainer.style.display = 'none';
    } else {
      wifiContainer.style.display = 'block';
      const wifiSsid = document.getElementById('wifi-select').value || '-';
      document.getElementById('preview-wifi-ssid').textContent = wifiSsid;
    }
  }

  // 绑定下一步按钮
  document.querySelectorAll('[id^="to-"]').forEach(btn => {
    btn.addEventListener('click', (e) => {
      const targetId = e.target.id.replace('to-', '');
      updateUI(parseInt(targetId));
    });
  });

  // 绑定上一步按钮
  document.querySelectorAll('[data-back]').forEach(btn => {
    btn.addEventListener('click', (e) => {
      const targetStep = parseInt(e.target.dataset.back);
      updateUI(targetStep);
    });
  });

  // 5. Pill 选择逻辑 (通用)
  document.querySelectorAll('.pill-group').forEach(group => {
    group.addEventListener('click', (e) => {
      if (e.target.classList.contains('pill')) {
        // 移除同组其他 active
        group.querySelectorAll('.pill').forEach(p => p.classList.remove('active'));
        e.target.classList.add('active');
        
        // 特殊逻辑：基础选择显示/隐藏
        if (group.id === 'iso-standard') {
          const isAdvanced = e.target.dataset.value === 'ISO20816';
          document.getElementById('foundation-group').style.display = isAdvanced ? 'block' : 'none';
          
          // 根据ISO标准更新机械类别选项
          updateIsoCategoryDropdown(isAdvanced);
        }
      }
    });
  });

  // 5.1 更新ISO类别下拉菜单
  function updateIsoCategoryDropdown(isAdvanced) {
    const dropdown = document.getElementById('iso-category-dropdown');
    let options = [];

    if (isAdvanced) {
      // ISO 20816 高级选项 - 包含LaTeX公式
      options = [
        { 
          value: '1', 
          label: '中大型工业电机 (Motor)', 
          formula: 'P > 15\\text{ kW}, \\ 120 \\sim 15000\\text{ RPM}' 
        },
        { 
          value: '2', 
          label: '卧式离心泵 (Horizontal Pump)', 
          formula: '\\text{独立轴承}, \\ P > 15\\text{ kW}' 
        },
        { 
          value: '3', 
          label: '立式旋转机械 (Vertical Machine)', 
          formula: 'P > 15\\text{ kW}, \\ \\text{垂直悬挂结构}' 
        },
        { 
          value: '4', 
          label: '高速透平机械 (High-speed Turbo)', 
          formula: 'n > 15000\\text{ RPM}' 
        }
      ];
    } else {
      // ISO 10816 通用选项
      options = [
        { value: '1', label: 'Class I', formula: '15\\text{--}75\\text{ kW}' },
        { value: '2', label: 'Class II', formula: '\\leq 300\\text{ kW}' },
        { value: '3', label: 'Class III/IV', formula: '> 300\\text{ kW}' }
       
      ];
    }

    // 生成下拉菜单HTML
    dropdown.innerHTML = options.map((opt, idx) => `
      <div class="dropdown-item p-3 hover:bg-slate-50 cursor-pointer border-b border-slate-100 last:border-b-0 transition-colors" data-value="${opt.value}" data-index="${idx}">
        <div class="font-medium text-slate-900">${opt.label}</div>
        ${opt.formula ? `<div class="text-sm text-slate-500 mt-1" id="formula-${opt.value}"></div>` : ''}
      </div>
    `).join('');

    // 用KaTeX渲染公式
    options.forEach(opt => {
      if (opt.formula) {
        try {
          const formulaContainer = document.getElementById(`formula-${opt.value}`);
          if (formulaContainer) {
            katex.render(`${opt.formula}`, formulaContainer, {
              throwOnError: false,
              displayMode: false
            });
          }
        } catch (e) {
          console.error('KaTeX render error:', e);
        }
      }
    });

    // 绑定选项点击事件
    dropdown.querySelectorAll('.dropdown-item').forEach(item => {
      item.addEventListener('click', (e) => {
        const value = item.dataset.value;
        const label = item.querySelector('.font-medium').textContent;
        document.getElementById('iso-category').value = value;
        document.getElementById('iso-category-label').textContent = label;
        dropdown.classList.add('hidden');
      });
    });
  }

  // 5.2 ISO类别下拉菜单控制
  const isoCategoryTrigger = document.getElementById('iso-category-trigger');
  const isoCategoryDropdown = document.getElementById('iso-category-dropdown');

  if (isoCategoryTrigger) {
    isoCategoryTrigger.addEventListener('click', () => {
      isoCategoryDropdown.classList.toggle('hidden');
    });
  }

  // 点击页面其他地方关闭下拉菜单
  document.addEventListener('click', (e) => {
    if (!e.target.closest('#iso-category-trigger') && !e.target.closest('#iso-category-dropdown')) {
      isoCategoryDropdown.classList.add('hidden');
    }
  });

  // 初始化ISO类别下拉菜单（默认为ISO 10816）
  updateIsoCategoryDropdown(false);

  // 加载并填充配置数据
  if (configData) {
    // 延迟填充，确保所有UI元素已初始化
    setTimeout(() => {
      populateFormData(configData);
    }, 0);
  }

  // 6. Range Slider 逻辑
  const rangeInput = document.getElementById('report-cycle');
  const rangeVal = document.getElementById('cycle-val');
  if (rangeInput && rangeVal) {
    rangeInput.addEventListener('input', (e) => {
      rangeVal.textContent = e.target.value;
    });
  }

  // 7. WiFi 逻辑
  const wifiBox = document.getElementById('wifi-box');
  const commTypeGroup = document.getElementById('comm-type');
  const refreshWifiBtn = document.getElementById('refresh-wifi');
  const wifiSelect = document.getElementById('wifi-select');

  // 监听通讯方式切换 (通过pill-group通用逻辑 + 额外的WiFi处理)
  if (commTypeGroup) {
    commTypeGroup.addEventListener('click', (e) => {
      if (e.target.classList.contains('pill')) {
        const isWifi = e.target.dataset.value === '2';
        if (wifiBox) wifiBox.style.display = isWifi ? 'block' : 'none';
        if (isWifi) simulateWifiScan();
      }
    });
  }

  // 7.1 WiFi 扫描完整流程
  async function scanWifiNetworks() {
    const processingMask = document.getElementById('processing-mask');
    const maskText = processingMask?.querySelector('.font-medium');
    const maskSubtext = processingMask?.querySelector('.text-slate-400');
    
    try {
      // 显示等待窗口
      if (processingMask) {
        processingMask.classList.remove('hidden');
        processingMask.classList.add('flex');
      }
      if (maskText) maskText.textContent = '正在扫描WiFi热点...';
      if (maskSubtext) maskSubtext.innerHTML = '<div class="flex items-center gap-2"><div class="animate-spin inline-block w-4 h-4 border-2 border-white border-t-transparent rounded-full"></div><span>请稍候</span></div>';

      // 1. 启动WiFi扫描
      console.log('Starting WiFi scan...');
      const startResponse = await fetch('/api/wifi-scan-start', { method: 'GET' });
      console.log(startResponse);
      if (!startResponse.ok) {
        throw new Error('Failed to start WiFi scan');
      }

      // 2. 轮询获取扫描结果（5次，每次间隔1秒）
      let wifiData = null;
      let attempts = 0;
      const maxAttempts = 5;

      while (attempts < maxAttempts) {
        attempts++;
        
        if (maskSubtext) maskSubtext.textContent = `查询中 (${attempts}/${maxAttempts})...`;
        
        const response = await fetch('/api/wifi-list');
        if (!response.ok) {
          throw new Error('Failed to fetch WiFi list');
        }

        const data = await response.json();
        
        // 检查是否处理完成
        if (data.status === 'processing') {
          if (attempts < maxAttempts) {
            // 等待2秒后重试（增加延迟）
            await new Promise(resolve => setTimeout(resolve, 2000));
            continue;
          }
        } else if (data.networks && Array.isArray(data.networks)) {
          wifiData = data.networks;
          if (wifiData.length > 0) {
            break;
          }
        }
      }

      if (!wifiData) {
        throw new Error('WiFi scan timeout or no data received');
      }

      // 3. 过滤信号弱的热点 (rssi < -75)，但如果没有强信号的则显示所有
      let filteredNetworks = wifiData.filter(network => {
        return network.rssi >= -75;
      });

      if (filteredNetworks.length === 0) {
        // 如果没有强信号网络，显示所有扫到的网络
        filteredNetworks = wifiData;
      }

      if (filteredNetworks.length === 0) {
        throw new Error('No available WiFi networks found');
      }

      // 4. 填充WiFi选择下拉菜单
      const wifiSelect = document.getElementById('wifi-select');
      if (wifiSelect) {
        // 按信号强度排序（从强到弱）
        filteredNetworks.sort((a, b) => b.rssi - a.rssi);

        wifiSelect.innerHTML = '<option value="">请选择 WiFi</option>';
        
        filteredNetworks.forEach(network => {
          const signalShength = getSignalQuality(network.rssi);
          const encLabel = network.enc ? ' 🔒' : '';
          const option = document.createElement('option');
          option.value = network.ssid;
          option.dataset.encrypted = network.enc ? '1' : '0';
          option.textContent = `${network.ssid} (${signalShength})${encLabel}`;
          wifiSelect.appendChild(option);
        });

        wifiSelect.disabled = false;
      }

      // 隐藏等待窗口
      if (processingMask) {
        processingMask.classList.add('hidden');
        processingMask.classList.remove('flex');
      }

      return true; // 成功

    } catch (error) {
      console.error('WiFi scan error:', error);
      
      // 显示错误信息
      if (maskText) maskText.textContent = '扫描失败';
      if (maskSubtext) maskSubtext.textContent = error.message || '请检查网络连接';
      
      // 3秒后隐藏
      setTimeout(() => {
        if (processingMask) {
          processingMask.classList.add('hidden');
          processingMask.classList.remove('flex');
        }
      }, 3000);

      // WiFi选择框禁用
      const wifiSelect = document.getElementById('wifi-select');
      if (wifiSelect) {
        wifiSelect.innerHTML = '<option value="">扫描失败，请重试</option>';
        wifiSelect.disabled = true;
      }

      return false; // 失败
    }
  }

  // 7.2 信号强度评级
  function getSignalQuality(rssi) {
    if (rssi >= -50) return '强';
    if (rssi >= -60) return '中强';
    if (rssi >= -70) return '中';
    return '弱';
  }

  // 7.3 模拟 WiFi 扫描入口
  function simulateWifiScan() {
    if (!wifiSelect) return;
    wifiSelect.innerHTML = '<option>正在扫描...</option>';
    wifiSelect.disabled = true;
    
    // 调用新的扫描流程
    scanWifiNetworks();
  }

  if (refreshWifiBtn) {
    refreshWifiBtn.addEventListener('click', simulateWifiScan);
  }

  // WiFi 密码框显示逻辑 - 仅当选择加密热点时显示
  wifiSelect?.addEventListener('change', (e) => {
    const pwdContainer = document.getElementById('wifi-password-container');
    const wifiPassword = document.getElementById('wifi-password');
    
    if (!pwdContainer) return;
    
    if (!e.target.value) {
      // 未选择
      pwdContainer.style.display = 'none';
      if (wifiPassword) wifiPassword.value = '';
    } else {
      // 检查选中项是否加密
      const selectedOption = e.target.options[e.target.selectedIndex];
      const isEncrypted = selectedOption.dataset.encrypted === '1';
      
      if (isEncrypted) {
        pwdContainer.style.display = 'block';
        if (wifiPassword) wifiPassword.focus();
      } else {
        pwdContainer.style.display = 'none';
        if (wifiPassword) wifiPassword.value = '';
      }
    }
  });

  // 8. 确认配置按钮
  const submitBtn = document.getElementById('submit-config');
  if (submitBtn) {
    submitBtn.addEventListener('click', async () => {
      // 验证WiFi配置
      const commTypeBtn = document.querySelector('#comm-type .pill.active');
      const commType = commTypeBtn?.dataset.value;
      
      if (commType === '2') {
        // 选择了WiFi
        const wifiSelect = document.getElementById('wifi-select');
        const wifiPassword = document.getElementById('wifi-password');
        
        if (!wifiSelect?.value) {
          alert('请选择一个WiFi热点');
          return;
        }
        
        // 检查是否需要密码
        const selectedOption = wifiSelect.options[wifiSelect.selectedIndex];
        const isEncrypted = selectedOption.dataset.encrypted === '1';
        
        if (isEncrypted && !wifiPassword?.value) {
          alert('此WiFi网络已加密，请输入密码');
          return;
        }
      }
      
      // 显示处理中的遮罩
      if (mask) {
        mask.classList.remove('hidden');
        mask.classList.add('flex');
      }
      
      // 调用API提交配置
      try {
        const isoStandardBtn = document.querySelector('#iso-standard .pill.active');
        const isoStandard = isoStandardBtn?.dataset.value || '';
        const isoCategory = document.getElementById('iso-category')?.value || '';
        
        const foundationBtn = document.querySelector('#foundation-select .pill.active');
        const isoFoundation = foundationBtn?.dataset.value || 'rigid';

        const deviceId = document.getElementById('device-id')?.value || '';
        const deviceName = document.getElementById('device-name')?.value || '';
        const rpm = document.getElementById('device-rpm')?.value || 1480;
        const years = document.getElementById('years-used')?.value || 0;

        const wifiSSID = document.getElementById('wifi-select')?.value || '';
        const wifiPassword = document.getElementById('wifi-password')?.value || '';
        
        const reportCycle = document.getElementById('report-cycle')?.value || 6;
        
        const detectFreqBtn = document.querySelector('#detect-frequency .pill.active');
        const detectInterval = detectFreqBtn?.dataset.value || 30;
        
        const commTypeBtn = document.querySelector('#comm-type .pill.active');
        const commType = commTypeBtn?.dataset.value || 1;

        const config = {
          iso: {
            standard: isoStandard,
            category: isoCategory,
            foundation: isoFoundation
          },
          deviceId: deviceId,
          deviceName: deviceName,
          rpm: parseInt(rpm) || 1480,
          years: parseFloat(years) || 0,
          host: "https://sentinel-cloud.com",
          detect_interval: parseInt(detectInterval) || 30,
          report_cycle: parseInt(reportCycle) || 6,
          comm_type: parseInt(commType) || 1,
          ble_enabled: true,
          wifi: {
            ssid: wifiSSID,
            pass: wifiPassword
          },
          configured: true
        };

        console.log('Sending config:', config);
        
        const response = await fetch('/api/save', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(config)
        });
        
        if (!response.ok) {
          throw new Error(`保存失败: ${response.status}`);
        }
        
        const result = await response.json();
        console.log('配置已提交:', result);
        alert('配置已成功提交！设备将立即重启并开始监测。');
      } catch (error) {
        console.error('Save error:', error);
        alert('保存失败: ' + error.message);
      } finally {
        if (mask) {
          mask.classList.add('hidden');
          mask.classList.remove('flex');
        }
      }
    });
  }
});