export interface IntentTemplate {
  id: string;
  intent: string;
  title: string;
  subtitle: string;
}

export const INTENT_TEMPLATES: IntentTemplate[] = [
  {
    id: 'home',
    intent: 'home',
    title: 'Home / Về home',
    subtitle: 'Return to home pose · Đưa robot về vị trí home',
  },
  {
    id: 'stop',
    intent: 'stop',
    title: 'Stop / Dừng',
    subtitle: 'Immediate supervised stop request · Yêu cầu dừng có giám sát',
  },
  {
    id: 'up_5cm',
    intent: 'move up 5 cm',
    title: 'Move up 5 cm / Nâng lên 5 cm',
    subtitle: 'Small vertical lift in base_link · Tịnh tiến đứng nhỏ trong base_link',
  },
  {
    id: 'down_2cm',
    intent: 'move down 2 cm',
    title: 'Move down 2 cm / Hạ xuống 2 cm',
    subtitle: 'Small downward move in base_link · Tịnh tiến xuống nhỏ trong base_link',
  },
  {
    id: 'joint_1_plus_5',
    intent: 'move joint 1 +5 deg',
    title: 'Joint 1 +5° / Khớp 1 +5°',
    subtitle: 'Conservative joint adjustment · Điều chỉnh khớp bảo thủ',
  },
  {
    id: 'wait_2s',
    intent: 'wait 2 s',
    title: 'Wait 2 s / Chờ 2 giây',
    subtitle: 'Pause sequence safely · Tạm dừng chuỗi an toàn',
  },
  {
    id: 'get_pose',
    intent: 'get pose',
    title: 'Get pose / Lấy pose',
    subtitle: 'Query current TCP pose · Truy vấn pose TCP hiện tại',
  },
];
