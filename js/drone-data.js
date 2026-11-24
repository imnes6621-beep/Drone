/**
 * Drone Builder — Expanded 12-Part Engineering Data, Compatibility Rules, Presets, Missions & Physics
 * Supports 12 real multirotor subsystems (Frame, Motor, Prop, ESC, Battery, FC, Camera, GPS, RX, Sensor, Payload, Safety)
 */

const DRONE_PARTS = {
  frame: {
    id: 'frame',
    num: '01',
    calloutCode: 'A · โครงโดรน/แขน',
    name: 'โครงโดรน',
    enName: 'Frame',
    lesson: 'โครงเป็นตัวรับน้ำหนักของทุกชิ้นส่วน ขนาดของโครง (เช่น 450 มม.) คือระยะทแยงระหว่างแกนมอเตอร์ ยิ่งโครงใหญ่ยิ่งใส่ใบพัดใหญ่และบรรทุกเพย์โหลดได้มากขึ้น แต่ก็หนักและมีโมเมนต์ความเฉื่อยสูงขึ้น — โครงคาร์บอนไฟเบอร์ 3K แข็งแกร่งแต่น้ำหนักเบา มีเพลตบน–ล่างประกบด้วยเสาอะลูมิเนียม CNC เพื่อลดแรงสั่นสะเทือน',
    options: [
      {
        id: 'frame_250',
        name: 'โครง 250 มม. (FPV เล็ก)',
        wheelbase: 250,
        weight: 96,
        price: 690,
        minPropSize: 5,
        maxPropSize: 5.5,
        maxMotorDiameter: 24,
        maxPayloadGrams: 320,
        mountSpec: 'รูยึด 16×16',
        chips: ['96 ก.', '฿690', 'ใบ 5"', 'มอเตอร์ ≤ 24 มม.', 'เพย์โหลด ≤ 320 ก.'],
        desc: 'เบา คล่องตัวสูง ตอบสนองไว เหมาะกับบินซิ่ง FPV แต่รับลมและน้ำหนักบรรทุกได้น้อย',
        lesson: 'โครง 250 มม. มีโมเมนต์ความเฉื่อยต่ำมาก ทำให้พลิกตัวไวและทำความเร็วสูง แต่ใส่ใบพัดได้ไม่เกิน 5 นิ้วและรับน้ำหนักเพิ่มได้จำกัด'
      },
      {
        id: 'frame_450',
        name: 'โครง 450 มม. (Aerial กลาง)',
        wheelbase: 450,
        weight: 178,
        price: 1190,
        minPropSize: 9,
        maxPropSize: 10.5,
        maxMotorDiameter: 26,
        maxPayloadGrams: 1100,
        mountSpec: 'รูยึด 16×16 / 19×19',
        recommended: true,
        badge: 'แนะนำ',
        chips: ['178 ก.', '฿1,190', 'ใบ 9–10"', 'มอเตอร์ ≤ 26 มม.', 'เพย์โหลด ≤ 1.1 กก.'],
        desc: 'สมดุลที่สุดสำหรับถ่ายภาพทางอากาศและผู้เริ่มต้น รับลมและกิมบอล 4K ได้ดี',
        lesson: 'โครง 450 มม. เป็นขนาดมาตรฐานยอดนิยม ใส่ใบพัด 10 นิ้วที่ประหยัดพลังงานได้พอดี และมีพื้นที่เพลตกลางรองรับ FC/ESC/GPS/LiDAR ครบชุด'
      },
      {
        id: 'frame_650',
        name: 'โครง 650 มม. (Heavy Survey)',
        wheelbase: 650,
        weight: 312,
        price: 1890,
        minPropSize: 12,
        maxPropSize: 15,
        maxMotorDiameter: 30,
        maxPayloadGrams: 2800,
        mountSpec: 'รูยึด 19×19 / 25×25',
        chips: ['312 ก.', '฿1,890', 'ใบ 12–15"', 'มอเตอร์ ≤ 30 มม.', 'เพย์โหลด ≤ 2.8 กก.'],
        desc: 'แขนท่อคาร์บอนขนาดใหญ่ ยกกล้อง Thermal/RTK หรือรอกส่งพัสดุได้ดีมาก นิ่งสู้ลมแรง',
        lesson: 'โครง 650 มม. ใช้แขนท่อคาร์บอนไฟเบอร์รับแรงบิดสูง เหมาะกับใบพัด 12 นิ้วและมอเตอร์รอบต่ำแรงบิดสูงสำหรับงานสำรวจและยกหนัก'
      },
      {
        id: 'frame_800_agri',
        name: 'โครง 800 มม. (Agri พับแขนได้)',
        wheelbase: 800,
        isAgriFrame: true,
        weight: 465,
        price: 3290,
        minPropSize: 14,
        maxPropSize: 18,
        maxMotorDiameter: 36,
        maxPayloadGrams: 5500,
        mountSpec: 'ข้อพับ CNC · รูยึด 25×25 / 30×30',
        badge: 'งานเกษตร',
        chips: ['465 ก.', '฿3,290', 'ใบ 14–18"', 'ข้อพับ CNC', 'เพย์โหลด ≤ 5.5 กก.'],
        desc: 'โครงเกษตรคาร์บอนพับแขนได้ แข็งแรงระดับอุตสาหกรรม รองรับถังพ่นปุ๋ยและใบพัด 16 นิ้ว',
        lesson: 'โดรนเกษตรต้องแบกถังปุ๋ยที่หนักหลายกิโลกรัม จึงใช้โครงท่อคาร์บอนหนาขนาด 800 มม. พร้อมข้อพับอะลูมิเนียม CNC เพื่อพับเก็บขนย้ายไปแปลงนาได้สะดวก'
      }
    ]
  },

  motor: {
    id: 'motor',
    num: '02',
    calloutCode: 'B · มอเตอร์ Brushless',
    name: 'มอเตอร์',
    enName: 'Brushless Motor',
    lesson: 'มอเตอร์ไร้แปรงถ่าน (Brushless Outrunner) แปลงไฟฟ้า 3 เฟสเป็นแรงหมุน ค่า KV คือรอบหมุนต่อแรงดัน 1V — KV สูงหมุนจัดเหมาะใบเล็ก ส่วน KV ต่ำให้แรงบิดมหาศาลขับใบใหญ่ — โดรน Quad วางมอเตอร์หมุนตามเข็ม (CW) 2 ตัว และทวนเข็ม (CCW) 2 ตัวสลับทแยงเพื่อหักล้างแรงบิด',
    options: [
      {
        id: 'motor_2306',
        name: '2306 · 1700KV (FPV รอบจัด)',
        kv: 1700,
        diameter: 23,
        weight: 124,
        price: 480,
        baseThrustPerMotor: 860,
        maxPropThrustMult: 0.85,
        maxAmpsPerMotor: 26,
        chips: ['124 ก. (×4)', '฿480', '1700KV', 'Ø 23 มม.', 'เหมาะใบ 5"'],
        desc: 'รอบจัด ตอบสนองฉับไว หน่วงรอบน้อยมาก เหมาะกับโครงเล็ก 250 มม.',
        lesson: 'มอเตอร์ 2306 1700KV เร่งและลดรอบได้ภายใน ~30 มิลลิวินาที ทำให้บินคล่องตัวมาก แต่กินกระแสพีคสูงจึงต้องใช้ ESC ≥ 30A'
      },
      {
        id: 'motor_2212',
        name: '2212 · 920KV (Aerial สมดุล)',
        kv: 920,
        diameter: 22,
        weight: 224,
        price: 390,
        baseThrustPerMotor: 880,
        maxPropThrustMult: 1.10,
        maxAmpsPerMotor: 14,
        recommended: true,
        badge: 'แนะนำ',
        chips: ['224 ก. (×4)', '฿390', '920KV', 'Ø 22 มม.', 'เหมาะใบ 9–10"'],
        desc: 'สมดุลที่สุดสำหรับโครงกลาง 450 มม. และใบพัด 10 นิ้ว ประหยัดไฟ เสถียร',
        lesson: 'มอเตอร์ 2212 920KV เป็นมาตรฐานโดรนถ่ายภาพ 450 มม. ให้แรงบิดพอดีสำหรับใบพัด 10 นิ้วโดยไม่ร้อนและใช้กระแสไม่สูงเกินไป'
      },
      {
        id: 'motor_2814',
        name: '2814 · 700KV (Heavy Torque)',
        kv: 700,
        diameter: 28,
        weight: 432,
        price: 720,
        baseThrustPerMotor: 1120,
        maxPropThrustMult: 1.32,
        maxAmpsPerMotor: 28,
        chips: ['432 ก. (×4)', '฿720', '700KV', 'Ø 28 มม.', 'โครง ≥ 650 มม.'],
        desc: 'แรงบิดสูงสำหรับงานสำรวจหนักและใบพัด 12 นิ้ว (ต้องใช้โครง ≥ 650 มม.)',
        lesson: 'ขดลวดทองแดงขนาดใหญ่ 700KV หมุนช้ากว่าแต่ให้แรงบิดขับใบ 12 นิ้วได้อย่างมีประสิทธิภาพ ลดเสียงดังและประหยัดไฟขณะบรรทุกหนัก'
      },
      {
        id: 'motor_3515',
        name: '3515 · 480KV (Agri Industrial)',
        kv: 480,
        diameter: 35,
        weight: 620,
        price: 1290,
        baseThrustPerMotor: 1320,
        maxPropThrustMult: 1.60,
        maxAmpsPerMotor: 42,
        badge: 'แรงบิดสูงสุด',
        chips: ['620 ก. (×4)', '฿1,290', '480KV', 'Ø 35 มม.', 'โครง 800 มม.'],
        desc: 'มอเตอร์เกษตรแรงบิดมหาศาล ยกถังปุ๋ยน้ำและใบพัด 16 นิ้วได้สบาย (โครง 800 มม. + ESC ≥ 45A)',
        lesson: 'มอเตอร์ 3515 480KV มีขนาดสเตเตอร์กว้างพิเศษ สร้างแรงยกได้มากกว่า 2.6 กก. ต่อตัว (รวมกว่า 10 กก.!) เหมาะกับโดรนเกษตรพ่นปุ๋ยโดยเฉพาะ'
      }
    ]
  },

  propeller: {
    id: 'propeller',
    num: '03',
    calloutCode: 'C · ใบพัด CW/CCW',
    name: 'ใบพัด',
    enName: 'Propeller',
    lesson: 'ใบพัดสร้างแรงยกจากหน้าตัดปีกเครื่องบิน (Airfoil) และมุมบิด (Pitch) — ใบ 2 แฉกประหยัดพลังงานกว่า ส่วนใบ 3 แฉกเกาะอากาศและตอบสนองไวกว่า และใบคาร์บอนไฟเบอร์แข็งแรงไม่บิดตัวเวลาเจอลมกระโชก — ต้องใส่คู่ CW/CCW ให้ตรงแกนมอเตอร์เสมอ',
    options: [
      {
        id: 'prop_5030',
        name: '5 นิ้ว · 3 แฉก โพลีคาร์บอเนต',
        size: 5,
        pitch: 3.0,
        blades: 3,
        material: 'polycarb',
        weight: 20,
        price: 180,
        thrustMultiplier: 0.72,
        efficiencyFactor: 0.94,
        chips: ['20 ก. (×4)', '฿180', '5 นิ้ว · 3 แฉก', 'Pitch 3.0'],
        desc: 'เล็ก คล่องตัว เหนียวทนแรงกระแทก ใช้กับโครง 250 มม.',
        lesson: 'ใบพัด 5 นิ้วแบบ 3 แฉกให้แรงเกาะอากาศสูงในวงเลี้ยวแคบ และเปลี่ยนรอบได้เร็วมาก เหมาะกับ FPV'
      },
      {
        id: 'prop_1045',
        name: '10 นิ้ว · 2 แฉก ไนลอน (ประหยัดไฟ)',
        size: 10,
        pitch: 4.5,
        blades: 2,
        material: 'nylon',
        weight: 48,
        price: 290,
        thrustMultiplier: 1.0,
        efficiencyFactor: 1.0,
        recommended: true,
        badge: 'แนะนำ',
        chips: ['48 ก. (×4)', '฿290', '10 นิ้ว · 2 แฉก', 'ประหยัดพลังงาน'],
        desc: 'ประสิทธิภาพสูงสุดต่อวัตต์ บินได้นาน เหมาะกับโครง 450 มม.',
        lesson: 'ใบพัด 2 แฉกมีแรงต้านหมุนต่ำกว่า 3 แฉก ทำให้ประหยัดแบตเตอรี่และลอยตัวได้นานขึ้นในงานถ่ายภาพ'
      },
      {
        id: 'prop_1045_tri',
        name: '10 นิ้ว · 3 แฉก คาร์บอนไฟเบอร์',
        size: 10,
        pitch: 4.8,
        blades: 3,
        material: 'carbon',
        weight: 66,
        price: 540,
        thrustMultiplier: 1.13,
        efficiencyFactor: 0.91,
        chips: ['66 ก. (×4)', '฿540', '10 นิ้ว · 3 แฉก', 'คาร์บอน · แรงยก +13%'],
        desc: 'แข็งแรงไม่บิดตัว แรงยกสูงและเบรกอากาศคมกว่า 2 แฉก แต่กินไฟเพิ่มขึ้นเล็กน้อย',
        lesson: 'ใบคาร์บอน 3 แฉกเพิ่มพื้นที่ปัดอากาศ ทำให้ไต่เร็วและสู้ลมกระโชกนิ่งกว่า แต่แลกกับภาระกระแสไฟที่สูงขึ้นประมาณ 9%'
      },
      {
        id: 'prop_1255',
        name: '12 นิ้ว · 2 แฉก คาร์บอนไฟเบอร์',
        size: 12,
        pitch: 5.5,
        blades: 2,
        material: 'carbon',
        weight: 84,
        price: 590,
        thrustMultiplier: 1.32,
        efficiencyFactor: 1.05,
        chips: ['84 ก. (×4)', '฿590', '12 นิ้ว · 2 แฉก', 'คาร์บอน · โครง ≥ 650 มม.'],
        desc: 'ใบคาร์บอนใหญ่แรงยกสูง เสียงทุ้มนุ่มนวล สำหรับโครง 650 มม.',
        lesson: 'ใบพัด 12 นิ้วกวาดอากาศพื้นที่กว้างกว่าใบ 10 นิ้วถึง 44% จึงสร้างแรงยกมหาศาลที่รอบต่ำและเกิด Ground Effect ชัดเจน'
      },
      {
        id: 'prop_1655',
        name: '16 นิ้ว · 2 แฉก พับได้คาร์บอน (Agri)',
        size: 16,
        pitch: 5.5,
        blades: 2,
        material: 'carbon',
        foldable: true,
        weight: 136,
        price: 1190,
        thrustMultiplier: 1.56,
        efficiencyFactor: 1.08,
        badge: 'งานเกษตร',
        chips: ['136 ก. (×4)', '฿1,190', '16 นิ้ว พับได้', 'คาร์บอน · โครง 800 มม.'],
        desc: 'ใบพัดเกษตรคาร์บอนพับได้ แรงกดลมลง (Downwash) กว้าง ช่วยพัดละอองปุ๋ยเกาะใบพืชทั่วถึง',
        lesson: 'โดรนเกษตรใช้ใบพัดพับได้ขนาด 16 นิ้วขึ้นไป เพื่อให้เกิดแรงลมกดลง (Downwash) ที่สม่ำเสมอช่วยดันละอองปุ๋ยแทรกซึมใต้ใบพืช'
      }
    ]
  },

  esc: {
    id: 'esc',
    num: '04',
    calloutCode: 'D · ESC 4-in-1',
    name: 'ESC',
    enName: 'Electronic Speed Controller',
    lesson: 'ESC (Electronic Speed Controller) แปลงคำสั่งดิจิทัล (DShot/PWM) จาก FC เป็นไฟฟ้า 3 เฟสขับมอเตอร์แต่ละตัว พร้อมครีบระบายความร้อนและตัวเก็บประจุกรองไฟกระชาก — ค่าพิกัดแอมป์ (A) ต้องสูงกว่ากระแสสูงสุดของมอเตอร์ มิฉะนั้นจะร้อนจัดหรือตัดไฟกลางอากาศ',
    options: [
      {
        id: 'esc_20a',
        name: '20A · Micro 4-in-1',
        rating: 20,
        weight: 28,
        price: 590,
        chips: ['28 ก.', '฿590', 'พิกัด 20A', 'เบาประหยัด'],
        desc: 'น้ำหนักเบา ราคาประหยัด เหมาะกับมอเตอร์ 2212 กินกระแสไม่สูง',
        lesson: 'ESC 20A เบาและคุ้มค่า แต่ห้ามใช้กับมอเตอร์ที่กินกระแสพีคเกิน 20A เพราะ MOSFET จะร้อนและตัดกำลังไฟ'
      },
      {
        id: 'esc_30a',
        name: '30A · 4-in-1 + Heatsink',
        rating: 30,
        weight: 42,
        price: 890,
        recommended: true,
        badge: 'แนะนำ',
        chips: ['42 ก.', '฿890', 'พิกัด 30A', '4-in-1 + ซิงค์'],
        desc: 'รองรับมอเตอร์ 2212 / 2306 / 2814 ได้สบาย มีระยะเผื่อความปลอดภัยดี',
        lesson: 'ESC 30A พร้อมครีบระบายความร้อนและคาปาซิเตอร์กรองไฟเป็นจุดสมดุลที่ปลอดภัยสำหรับโดรน 250–650 มม.'
      },
      {
        id: 'esc_45a',
        name: '45A · Heavy Duty 6S',
        rating: 45,
        weight: 60,
        price: 1390,
        chips: ['60 ก.', '฿1,390', 'พิกัด 45A', 'รองรับ 6S'],
        desc: 'รองรับกระแสกระชากสูงและแบต 6S เหมาะกับงานยกหนักและมอเตอร์แรงบิดสูง',
        lesson: 'ESC 45A ใช้ MOSFET ขนาดใหญ่ทนแรงดัน 6S (25.2V) และกระแสพีคสูงได้ต่อเนื่องโดยไม่เกิดความร้อนสะสม'
      },
      {
        id: 'esc_60a',
        name: '60A · Industrial Waterproof (IP67)',
        rating: 60,
        waterproof: true,
        weight: 86,
        price: 1990,
        badge: 'กันน้ำ/เกษตร',
        chips: ['86 ก.', '฿1,990', 'พิกัด 60A', 'กันละอองน้ำ IP67'],
        desc: 'เคลือบกันน้ำระดับ IP67 รองรับมอเตอร์เกษตร 3515 และบินต่อเนื่องกลางแดดได้ดีเยี่ยม',
        lesson: 'โดรนพ่นปุ๋ยต้องสัมผัสความชื้นและสารละลายตลอดเวลา ESC แบบเคลือบกันน้ำ IP67 และรับกระแสได้ 60A จึงช่วยป้องกันไฟลัดวงจร'
      }
    ]
  },

  battery: {
    id: 'battery',
    num: '05',
    calloutCode: 'E · แบตเตอรี่',
    name: 'แบตเตอรี่',
    enName: 'Battery Pack (LiPo / Li-Ion)',
    lesson: 'แบตเตอรี่คือแหล่งพลังงานและเป็นชิ้นส่วนที่หนักที่สุด — ค่า S คือจำนวนเซลล์ (แรงดันไฟ), mAh คือความจุ และ C-Rating คืออัตราการจ่ายกระแสพีค — LiPo จ่ายไฟแรงดันไม่ตกเวลาไต่หนัก ส่วน Li-Ion เบาและบินได้นานกว่าแต่ C-Rating ต่ำกว่าจึงแรงดันตกเร็วเมื่อเร่งเต็มกำลัง',
    options: [
      {
        id: 'bat_3s2200',
        name: 'LiPo 3S · 2200mAh (45C)',
        chemistry: 'lipo',
        cRating: 45,
        cells: 3,
        voltage: 11.1,
        capacity: 2200,
        weight: 195,
        price: 650,
        chips: ['195 ก.', '฿650', '3S · 11.1V', '2200mAh · 45C'],
        desc: 'น้ำหนักเบา ปลอดภัย เหมาะกับโครง 250 มม. และฝึกบินระยะสั้น',
        lesson: 'แบต LiPo 3S (11.1V) เบาและคล่องตัว แต่มีแรงดันและความจุสะสมน้อยกว่า 4S'
      },
      {
        id: 'bat_4s3300',
        name: 'LiPo 4S · 3300mAh (45C)',
        chemistry: 'lipo',
        cRating: 45,
        cells: 4,
        voltage: 14.8,
        capacity: 3300,
        weight: 392,
        price: 1450,
        recommended: true,
        badge: 'แนะนำ',
        chips: ['392 ก.', '฿1,450', '4S · 14.8V', '3300mAh · 45C'],
        desc: 'สมดุลที่สุดระหว่างแรงดัน ความจุ และการจ่ายกระแสไม่ตกขณะไต่หนัก',
        lesson: 'LiPo 4S 45C จ่ายกระแสต่อเนื่องได้สูงถึง ~148A จึงเกิด Voltage Sag น้อยเวลาไต่ความสูงเร็วหรือสู้ลมแรง'
      },
      {
        id: 'bat_4s4200_liion',
        name: 'Li-Ion 4S · 4200mAh (15C Endurance)',
        chemistry: 'liion',
        cRating: 15,
        cells: 4,
        voltage: 14.8,
        capacity: 4200,
        weight: 315,
        price: 1690,
        badge: 'บินนานพิเศษ',
        chips: ['315 ก.', '฿1,690', 'Li-Ion 21700', '4200mAh · 15C'],
        desc: 'เซลล์ทรงกระบอก 21700 เบากว่าและบินได้นานกว่า LiPo มาก แต่ C-Rating ต่ำกว่า (แรงดันตกไวถ้าไต่กระชาก)',
        lesson: 'แบต Li-Ion มีความหนาแน่นพลังงานสูงกว่า LiPo จึงเบากว่าและบินสำรวจได้นานกว่า ~35% แต่ความต้านทานภายในสูงกว่า ทำให้เกิด Voltage Sag มากกว่าตอนเร่งเต็มกำลัง'
      },
      {
        id: 'bat_6s5200',
        name: 'LiPo 6S · 5200mAh (50C)',
        chemistry: 'lipo',
        cRating: 50,
        cells: 6,
        voltage: 22.2,
        capacity: 5200,
        weight: 680,
        price: 2890,
        chips: ['680 ก.', '฿2,890', '6S · 22.2V', '5200mAh · 50C'],
        desc: 'แรงดันสูง 22.2V จ่ายไฟนิ่งมากสำหรับโดรนยกหนัก 650 มม. (หนักเกือบ 7 ขีด)',
        lesson: 'แบต 6S ให้กำลังวัตต์สูงที่กระแสต่ำลง (P = V×I) ช่วยลดความร้อนในสายไฟและ ESC แต่มีน้ำหนักมากต้องใช้มอเตอร์แรงบิดสูง'
      },
      {
        id: 'bat_6s10000',
        name: 'LiPo 6S · 10000mAh (Smart Agri)',
        chemistry: 'lipo',
        cRating: 35,
        smartPack: true,
        cells: 6,
        voltage: 22.2,
        capacity: 10000,
        weight: 1180,
        price: 4890,
        badge: 'งานเกษตร',
        chips: ['1,180 ก.', '฿4,890', '6S · 22.2V', '10000mAh Smart'],
        desc: 'แพ็กอัจฉริยะเคสแข็งพร้อมไฟ LED แสดงระดับแบต ความจุมหาศาลสำหรับโดรนพ่นปุ๋ย 800 มม.',
        lesson: 'แบต 6S 10000mAh เก็บพลังงานถึง 222Wh เพียงพอสำหรับบินแบกถังปุ๋ยพ่นแปลงเกษตรหลายรอบ แต่หนักถึง 1.18 กก. จึงเหมาะกับโครง 650–800 มม. เท่านั้น'
      }
    ]
  },

  fc: {
    id: 'fc',
    num: '06',
    calloutCode: 'F · FC + IMU Stack',
    name: 'ชุดควบคุมการบิน',
    enName: 'Flight Controller',
    lesson: 'Flight Controller (FC) คือสมองของโดรน อ่านข้อมูลไจโรสโคป (Gyro), มาตรความเร่ง (Accelerometer) และบารอมิเตอร์ (Barometer) แล้วคำนวณลูป PID สั่งมอเตอร์ทั้ง 4 ตัวหลายร้อยครั้งต่อวินาที — บอร์ดระดับอุตสาหกรรมใช้ Dual-IMU ไจโรคู่ช่วยกรองแรงสั่นและต้านลมกระโชกได้นิ่งมาก',
    options: [
      {
        id: 'fc_f4',
        name: 'F4 · Basic (Gyro เดี่ยว)',
        mcu: 'STM32F405',
        hasBaro: false,
        dualImu: false,
        weight: 18,
        price: 890,
        chips: ['18 ก.', '฿890', 'Gyro 6-Axis', 'ไม่มี Barometer'],
        desc: 'คุมการทรงตัวพื้นฐาน ไม่มีบารอมิเตอร์ (เวลาเอียงเลี้ยวแรง ๆ ความสูงจะตกเล็กน้อย ต้องเลี้ยงคันเร่งเอง)',
        lesson: 'บอร์ด F4 ไม่มีบารอมิเตอร์วัดความดันอากาศ จึงไม่สามารถล็อกความสูงอัตโนมัติได้ เหมาะกับการฝึกควบคุมคันเร่งแบบ FPV จริง'
      },
      {
        id: 'fc_f7',
        name: 'F7 · Pro + Barometer',
        mcu: 'STM32F722',
        hasBaro: true,
        dualImu: false,
        weight: 26,
        price: 1650,
        recommended: true,
        badge: 'แนะนำ',
        chips: ['26 ก.', '฿1,650', 'Barometer', 'Altitude Hold'],
        desc: 'ประมวลผลไว พร้อมบารอมิเตอร์ล็อกความสูงนิ่งและชดเชยแรงยกขณะเอียงตัวอัตโนมัติ',
        lesson: 'บอร์ด F7 มีบารอมิเตอร์ความละเอียดสูง ช่วยเบรกและล็อกระดับความสูงได้นิ่งทันทีที่ปล่อยปุ่ม Space/Shift'
      },
      {
        id: 'fc_h7',
        name: 'H7 · Dual-IMU Industrial',
        mcu: 'STM32H743',
        hasBaro: true,
        dualImu: true,
        weight: 38,
        price: 2890,
        badge: 'กันลมนิ่งสุด',
        chips: ['38 ก.', '฿2,890', 'Dual-IMU', 'Baro + กรองลมกระโชก'],
        desc: 'ชิป H7 ความเร็วสูง + ไจโรคู่พร้อมซับแรงสั่นสะเทือน ลดอาการสั่นจากลมกระโชกได้กว่า 55%',
        lesson: 'FC ระดับอุตสาหกรรมอย่าง H7 ใช้เซนเซอร์ IMU สองตัวทำงานสำรองกัน (Redundancy) หากตัวหนึ่งสั่นหรือผิดพลาดอีกตัวจะช่วยกรองสัญญาณทันที'
      }
    ]
  },

  camera: {
    id: 'camera',
    num: '07',
    calloutCode: 'G · กล้อง/เซนเซอร์ภาพ',
    name: 'กล้อง / เซนเซอร์ภาพ',
    enName: 'Camera / Optical Sensor',
    lesson: 'กล้องและเซนเซอร์ภาพคือดวงตาของโดรน — กล้อง FPV เบาและเอียงตามตัวลำ, กล้อง 4K Gimbal มีมอเตอร์ 3 แกนรักษาเส้นขอบฟ้าให้ตรงตลอดเวลา, ส่วนงานเกษตรและวิศวกรรมใช้กล้อง Multispectral (NDVI) วัดสุขภาพพืช หรือกล้อง Thermal ตรวจจับจุดร้อน',
    options: [
      {
        id: 'cam_none',
        name: 'ไม่ติดกล้อง',
        type: 'none',
        weight: 0,
        price: 0,
        payloadScore: 0,
        chips: ['0 ก.', '฿0', 'เบาที่สุด'],
        desc: 'ลดน้ำหนักและแรงต้านลมต่ำสุด เหมาะกับฝึกบินพื้นฐานเท่านั้น',
        lesson: 'ไม่ติดกล้องทำให้ตัวเครื่องเบาที่สุด บินได้นานขึ้น แต่ไม่สามารถทำภารกิจถ่ายภาพหรือสำรวจได้'
      },
      {
        id: 'cam_light',
        name: 'กล้อง FPV เบา + VTX 5.8GHz',
        type: 'fpv',
        weight: 40,
        price: 1290,
        payloadScore: 40,
        chips: ['40 ก.', '฿1,290', 'FPV มุมกว้าง', 'เอียงตามตัวลำ'],
        desc: 'กล้อง FPV น้ำหนักเบาพร้อมเสา VTX ภาพสดเอียงตามตัวเครื่องจริง เหมาะกับซิ่งและสำรวจเบื้องต้น',
        lesson: 'กล้อง FPV ยึดติดกับโครงโดยตรงพร้อมมุมเงย ~20° จึงเบาและทนแรงกระแทก แต่ภาพจะเอียงตามการบังคับทุกแกน'
      },
      {
        id: 'cam_4k_gimbal',
        name: 'กล้อง 4K + 3-Axis Gimbal',
        type: 'gimbal_4k',
        weight: 128,
        price: 3590,
        payloadScore: 128,
        recommended: true,
        badge: 'แนะนำ',
        chips: ['128 ก.', '฿3,590', '4K 60fps', 'กิมบอลกันสั่น 3 แกน'],
        desc: 'คมชัดระดับ 4K พร้อมกิมบอลมอเตอร์รักษาเส้นขอบฟ้าตรงนิ่งทั้งมุมมอง 3D และ FPV',
        lesson: 'กิมบอล 3 แกนมีมอเตอร์ไร้แปรงถ่านขนาดจิ๋วหมุนชดเชยมุมก้ม-เงยและเอียงของตัวลำ ทำให้ภาพนิ่งเรียบแม้บินสู้ลม'
      },
      {
        id: 'cam_ndvi',
        name: 'กล้อง Multispectral NDVI (เกษตร)',
        type: 'ndvi',
        weight: 155,
        price: 5490,
        payloadScore: 155,
        badge: 'งานเกษตร',
        chips: ['155 ก.', '฿5,490', '4 เลนส์สเปกตรัม', 'ตรวจพืชขาดปุ๋ย'],
        desc: 'ชุดกล้อง 4 เลนส์ (R/G/Red-Edge/NIR) พร้อมกิมบอล วิเคราะห์สุขภาพพืชและจุดขาดปุ๋ยในแปลงเกษตร',
        lesson: 'กล้อง Multispectral วัดการสะท้อนแสงย่านใกล้อินฟราเรด (NIR) เพื่อคำนวณดัชนี NDVI ทำให้เห็นจุดที่พืชขาดปุ๋ยก่อนมองเห็นด้วยตาเปล่า'
      },
      {
        id: 'cam_thermal',
        name: 'กล้อง Thermal + 4K Dual Gimbal',
        type: 'thermal',
        weight: 210,
        price: 6990,
        payloadScore: 210,
        badge: 'วิศวกรรม/กู้ภัย',
        chips: ['210 ก.', '฿6,990', 'อินฟราเรดความร้อน', 'กิมบอลคู่ 4K'],
        desc: 'กล้องถ่ายภาพความร้อนคู่กับ 4K สำหรับตรวจแผงโซลาร์เซลล์ร้อนผิดปกติ สายส่งไฟ และเฝ้าระวังไฟป่า',
        lesson: 'กล้อง Thermal ใช้เลนส์เจอร์เมเนียมตรวจจับรังสีความร้อน ช่วยหาแผงโซลาร์ที่เสื่อม (Hotspot) หรือจุดไฟป่าได้อย่างแม่นยำ'
      }
    ]
  },

  gps: {
    id: 'gps',
    num: '08',
    calloutCode: 'H · โมดูล GPS/RTK',
    name: 'GPS',
    enName: 'GPS / GNSS Module',
    lesson: 'โมดูล GPS และเข็มทิศแม่เหล็ก (Compass) บอกพิกัดและทิศทางบนโลก ทำให้โดรนเปิดโหมด GPS Hold เอียงตัวสู้ลมตรึงตำแหน่งได้เอง และบินกลับจุดเริ่มต้น (RTH) เมื่อกดปุ่มหรือสัญญาณรีโมตขาด',
    options: [
      {
        id: 'gps_none',
        name: 'ไม่มี GPS',
        hasGps: false,
        isRtk: false,
        weight: 0,
        price: 0,
        chips: ['0 ก.', '฿0', 'บินแมนนวล ATTI'],
        desc: 'ไม่มีระบบนำทาง เครื่องจะไหลตามลมเมื่อปล่อยมือและใช้ RTH ไม่ได้',
        lesson: 'ถ้าไม่มี GPS โดรนจะบินในโหมด ATTI ไหลตามลมและแรงเฉื่อย ต้องคอยบังคับแก้ลมด้วยตัวเอง'
      },
      {
        id: 'gps_m8n',
        name: 'GPS M8N + Compass',
        hasGps: true,
        isRtk: false,
        weight: 24,
        price: 760,
        recommended: true,
        badge: 'แนะนำ',
        chips: ['24 ก.', '฿760', 'Position Hold (H)', 'RTH กลับฐาน (R)'],
        desc: 'ล็อกตำแหน่งกลางอากาศสู้ลมอัตโนมัติ (คลาดเคลื่อน ~0.3 ม.) และบินกลับลานจอดได้',
        lesson: 'GPS M8N รับดาวเทียมหลายระบบพร้อมเข็มทิศบนเสาสูงพ้นสนามแม่เหล็กมอเตอร์ ช่วยให้ล็อกตำแหน่งและบิน RTH ได้เสถียร'
      },
      {
        id: 'gps_rtk',
        name: 'Dual-Freq GPS + RTK Survey',
        hasGps: true,
        isRtk: true,
        weight: 38,
        price: 2950,
        badge: 'แม่นยำระดับ ซม.',
        chips: ['38 ก.', '฿2,950', 'RTK ±2 ซม.', 'ลงจอดแม่นยำสูง'],
        desc: 'เสาเห็ด RTK สองย่านความถี่ ล็อกพิกัดนิ่งระดับเซนติเมตรสำหรับงานเกษตรพ่นปุ๋ยและวิศวกรรม',
        lesson: 'ระบบ RTK (Real-Time Kinematic) ใช้ค่าปรับแก้จากสถานีฐาน ลดความคลาดเคลื่อนจากระดับเมตรเหลือเพียง 2–3 เซนติเมตร'
      }
    ]
  },

  rx: {
    id: 'rx',
    num: '09',
    calloutCode: 'I · ตัวรับสัญญาณ RX',
    name: 'ตัวรับสัญญาณรีโมต',
    enName: 'RC Receiver (RX)',
    lesson: 'ตัวรับสัญญาณวิทยุ (RX) รับคำสั่งสติ๊กจากรีโมตนักบินส่งเข้า FC และส่งค่า Telemetry (แรงดันแบต/RSSI) กลับหน้าจอ — หากบินไกลเกินระยะคลื่น สัญญาณจะเริ่มขาดหายและระบบ Failsafe จะสั่งบินกลับฐานอัตโนมัติ (ถ้ามี GPS)',
    options: [
      {
        id: 'rx_24g',
        name: '2.4GHz Standard SBUS',
        rxType: '24g',
        rangeMeters: 165,
        failsafeRth: false,
        weight: 10,
        price: 390,
        chips: ['10 ก.', '฿390', 'ระยะ ~165 ม.', 'เสาหนวดคู่ V-Dipole'],
        desc: 'ตัวรับพื้นฐานระยะใกล้ถึงกลาง หากบินไกลเกิน ~165 ม. สัญญาณจะอ่อนและควบคุมหน่วงลง',
        lesson: 'คลื่น 2.4GHz มาตรฐานมีระยะควบคุมจำกัดและถูกรบกวนได้ง่ายกว่าคลื่นความถี่ต่ำเมื่อบินไกลจากจุดปล่อย'
      },
      {
        id: 'rx_elrs',
        name: 'ELRS 915MHz Long-Range',
        rxType: 'elrs',
        rangeMeters: 340,
        failsafeRth: true,
        weight: 14,
        price: 790,
        recommended: true,
        badge: 'แนะนำ',
        chips: ['14 ก.', '฿790', 'ระยะ 340+ ม.', 'เสา Immortal-T', 'Failsafe RTH'],
        desc: 'คลื่น 915MHz ทะลุทะลวงไกลครอบคลุมทั้งหุบเขา ความหน่วงต่ำมาก พร้อม Failsafe กลับฐานอัตโนมัติ',
        lesson: 'ExpressLRS 915MHz ใช้เทคนิค LoRa ให้ระยะบินไกลกว่า 2.4GHz หลายเท่าพร้อมส่งสัญญาณกลับได้ไว และสั่ง RTH อัตโนมัติเมื่อหลุดระยะ'
      },
      {
        id: 'rx_dual',
        name: 'Dual-Band 2.4G/900MHz Diversity',
        rxType: 'dual',
        rangeMeters: 480,
        failsafeRth: true,
        weight: 24,
        price: 1490,
        badge: 'อุตสาหกรรม',
        chips: ['24 ก.', '฿1,490', 'ระยะ 480 ม.', '2 ย่านความถี่สำรอง', 'Telemetry เต็มระบบ'],
        desc: 'รับสัญญาณพร้อมกัน 2 ย่านความถี่ สัญญาณ RSSI เต็มตลอดทั้งแมพ เหมาะกับงานสำรวจและเกษตร',
        lesson: 'ระบบ Dual-Band Diversity มีเสาสองย่านความถี่ทำงานคู่กัน หากคลื่นหนึ่งถูกรบกวนใกล้เสา 5G อีกย่านจะรับช่วงทันทีโดยไม่เสียการควบคุม'
      }
    ]
  },

  sensor: {
    id: 'sensor',
    num: '10',
    calloutCode: 'J · เซนเซอร์/LiDAR',
    name: 'เซนเซอร์ช่วยบิน / LiDAR',
    enName: 'Obstacle & Altitude Sensor',
    lesson: 'เซนเซอร์ช่วยบินเสริมความปลอดภัยให้โดรนยุคใหม่ — Optical Flow + Sonar ใต้ท้องช่วยล็อกตำแหน่งนิ่งในระดับต่ำ (< 6 ม.) แม้ไม่มี GPS ส่วน LiDAR 360° ยิงเลเซอร์วัดระยะสิ่งกีดขวางรอบตัวเพื่อช่วยเบรกชะลออัตโนมัติก่อนชนอาคารหรือต้นไม้',
    options: [
      {
        id: 'sensor_none',
        name: 'ไม่ติดตั้งเซนเซอร์เสริม',
        hasOpticalFlow: false,
        hasLidar: false,
        weight: 0,
        price: 0,
        chips: ['0 ก.', '฿0', 'ไม่มีระบบกันชน'],
        desc: 'ประหยัดน้ำหนักสุด เหมาะกับนักบิน FPV ที่ต้องการบินลอดช่องแคบเองโดยไม่ถูกเบรก',
        lesson: 'โดรนแข่ง FPV มักไม่ติดเซนเซอร์กันชนเพื่อลดน้ำหนักและไม่ให้ระบบเบรกอัตโนมัติขัดขวางการบินลอดช่องแคบ'
      },
      {
        id: 'sensor_optflow',
        name: 'Optical Flow + Sonar ใต้ท้อง',
        hasOpticalFlow: true,
        hasLidar: false,
        weight: 18,
        price: 890,
        recommended: true,
        badge: 'แนะนำ',
        chips: ['18 ก.', '฿890', 'VPS ล็อกต่ำ < 6 ม.', 'ลงจอดนุ่มนวล'],
        desc: 'กล้องและโซนาร์ใต้ท้องช่วยเบรกการไหลแนวนอนในระยะต่ำกว่า 6 ม. แม้อยู่ในโหมด ATTI / ในโรงเก็บ',
        lesson: 'ระบบ Vision Positioning (Optical Flow + Sonar) จับภาพพื้นผิวใต้ท้องโดรน ช่วยให้ลอยตัวนิ่งมากเวลาบินต่ำหรือลงจอดในอาคารที่อับสัญญาณ GPS'
      },
      {
        id: 'sensor_lidar',
        name: '360° LiDAR + Radar กันชน',
        hasOpticalFlow: true,
        hasLidar: true,
        weight: 62,
        price: 2950,
        badge: 'กันชนอัตโนมัติ',
        chips: ['62 ก.', '฿2,950', 'LiDAR หมุน 360°', 'เบรกก่อนชนอัตโนมัติ', 'VPS ใต้ท้อง'],
        desc: 'ป้อมเลเซอร์ LiDAR หมุน 360° ตรวจจับตึก/เสา/ต้นไม้แล้วเบรกชะลออัตโนมัติ + รักษาระดับพื้นแม่นยำ',
        lesson: 'LiDAR ยิงลำแสงเลเซอร์วัดระยะสิ่งกีดขวางรอบทิศทาง ทำให้ระบบ Flight Controller สั่งเบรกต้านก่อนพุ่งชนอาคารหรือเสาไฟฟ้าได้จริง'
      }
    ]
  },

  payload: {
    id: 'payload',
    num: '11',
    calloutCode: 'K · ถังปุ๋ย/เพย์โหลด',
    name: 'เพย์โหลดภารกิจ / เกษตร',
    enName: 'Mission Payload / Sprayer',
    lesson: 'อุปกรณ์ปฏิบัติภารกิจใต้ท้องโดรน เช่น ถังพ่นปุ๋ยน้ำ ถังหว่านเมล็ด หรือรอกส่งพัสดุ — ในโลกจริงน้ำหนักของปุ๋ยและพัสดุเปลี่ยนแปลงตลอดเวลา (Dynamic Mass) ตอนถังเต็มโดรนจะหนักและกินไฟสูง พอพ่นหรือปลดของออก น้ำหนักจะเบาลงและคล่องตัวขึ้นทันที!',
    options: [
      {
        id: 'payload_none',
        name: 'ไม่ติดอุปกรณ์บรรทุกเสริม',
        payloadType: 'none',
        tankKg: 0,
        dryWeight: 0,
        weight: 0,
        price: 0,
        recommended: true,
        badge: 'มาตรฐาน',
        chips: ['0 ก.', '฿0', 'บินเบาคล่องตัว'],
        desc: 'เหมาะสำหรับถ่ายภาพ สำรวจ และฝึกบินทั่วไปโดยไม่ต้องแบกน้ำหนักถัง',
        lesson: 'การถอดอุปกรณ์บรรทุกที่ไม่จำเป็นออกช่วยให้ค่า T/W สูงและยืดเวลาบินได้มากที่สุด'
      },
      {
        id: 'payload_drop',
        name: 'รอกและกล่องพัสดุกู้ภัย 500 ก.',
        payloadType: 'cargo',
        tankKg: 0.5,
        dryWeight: 110,
        weight: 610,
        price: 1490,
        chips: ['610 ก. (กล่อง 500 ก.)', '฿1,490', 'กด F ปลดพัสดุ', 'โครง ≥ 450 มม.'],
        desc: 'แขวนกล่องเวชภัณฑ์ใต้ท้อง กดปุ่ม F กลางอากาศเพื่อปลดพัสดุแล้วตัวลำจะเบาขึ้นทันที',
        lesson: 'โดรนส่งเวชภัณฑ์ต้องคำนวณ T/W ตอนมีของเต็ม และรับมือกับการเปลี่ยนจุดศูนย์ถ่วงทันทีที่ปลดกล่องออก'
      },
      {
        id: 'payload_sprayer',
        name: 'ถังพ่นปุ๋ยน้ำ 1.2 ลิตร + หัวฉีดคู่',
        payloadType: 'sprayer',
        tankKg: 1.2,
        dryWeight: 220,
        weight: 1420,
        price: 2490,
        badge: 'พ่นปุ๋ย (กด F)',
        chips: ['1,420 ก. (น้ำ 1.2 กก.)', '฿2,490', 'กด F พ่นละออง 3D', 'เติมได้ที่แท่นแปลงเกษตร'],
        desc: 'ถังปุ๋ยน้ำพร้อมปั๊มและก้านหัวฉีด กด F พ่นละอองเหนือแปลง Zone A–D น้ำหนักลดจริงตามน้ำในถัง และลงเติมได้ที่แท่น Refill!',
        lesson: 'โดรนเกษตรพ่นปุ๋ยมีน้ำหนักเปลี่ยนตลอดเที่ยวบิน (Dynamic Mass) เมื่อพ่นน้ำปุ๋ยออกหมด น้ำหนักหายไปกว่า 1.2 กก. ทำให้ประหยัดไฟและบินคล่องขึ้นชัดเจน'
      },
      {
        id: 'payload_spreader',
        name: 'ถังหว่านปุ๋ยเม็ด/เมล็ดพันธุ์ 1.5 กก.',
        payloadType: 'spreader',
        tankKg: 1.5,
        dryWeight: 260,
        weight: 1760,
        price: 2890,
        badge: 'หว่านเม็ด (กด F)',
        chips: ['1,760 ก. (เม็ด 1.5 กก.)', '฿2,890', 'จานเหวี่ยง 360°', 'เติมได้ที่แท่นแปลงเกษตร'],
        desc: 'ถังกรวยพร้อมจานหมุนเหวี่ยงปุ๋ยเม็ด/เมล็ดพันธุ์กระจายวงกว้าง กด F หว่านได้จริงบนแปลงเกษตร',
        lesson: 'ชุดหว่านเม็ดใช้จานหมุนแรงเหวี่ยงหนีศูนย์กลางกระจายปุ๋ยเม็ดได้กว้างกว่าหัวฉีดน้ำ แต่มีน้ำหนักรวมสูงมาก ต้องใช้โครง 650–800 มม. และมอเตอร์แรงบิดสูง'
      }
    ]
  },

  safety: {
    id: 'safety',
    num: '12',
    calloutCode: 'L · ขาตั้ง/กันชน/ร่ม',
    name: 'ขาตั้งและอุปกรณ์ป้องกัน',
    enName: 'Landing Gear & Safety',
    lesson: 'อุปกรณ์ความปลอดภัยช่วยปกป้องโดรนและผู้คนรอบข้าง — การ์ดกันใบพัดช่วยป้องกันการชนผนัง/กิ่งไม้ ขาตั้งสูงช่วยยกท้องเครื่องให้พ้นถังปุ๋ยและซับแรงกระแทก ส่วนร่มชูชีพฉุกเฉินช่วยกางชะลอความเร็วตกอัตโนมัติเมื่อแบตหมดหรือมอเตอร์ดับกลางอากาศ',
    options: [
      {
        id: 'safety_standard',
        name: 'ขาตั้งคาร์บอนมาตรฐาน',
        gearType: 'standard',
        hasGuards: false,
        hasParachute: false,
        clearanceExtra: 0,
        weight: 32,
        price: 290,
        recommended: true,
        badge: 'แนะนำ',
        chips: ['32 ก.', '฿290', 'เบา ต้านลมน้อย'],
        desc: 'ขาตั้งสกีคาร์บอนน้ำหนักเบา แรงต้านลมต่ำ เหมาะกับการบินทั่วไป',
        lesson: 'ขาตั้งแบบสกีคาร์บอนช่วยกระจายน้ำหนักลงจอดบนพื้นหญ้าและลานปูนได้ดีโดยเพิ่มน้ำหนักน้อยมาก'
      },
      {
        id: 'safety_guards',
        name: 'การ์ดกันใบพัด 360° (Prop Guards)',
        gearType: 'guards',
        hasGuards: true,
        hasParachute: false,
        clearanceExtra: 0,
        weight: 78,
        price: 690,
        chips: ['78 ก.', '฿690', 'กันใบชนผนัง/ต้นไม้', 'แรงต้านลม +12%'],
        desc: 'วงแหวนครอบใบพัดทั้ง 4 มุม ลดโอกาสพังจากการชนข้างได้ 65% เหมาะสำหรับมือใหม่และบินในที่แคบ',
        lesson: 'การ์ดกันใบพัดช่วยป้องกันอันตรายจากใบพัดและกันใบแตกเมื่อเฉี่ยวกำแพง แต่จะเพิ่มน้ำหนักที่ปลายแขนและเพิ่มแรงต้านลมเล็กน้อย'
      },
      {
        id: 'safety_tall_agri',
        name: 'ขาตั้งสูง Heavy-Duty + โช้กซับแรง',
        gearType: 'tall',
        hasGuards: false,
        hasParachute: false,
        clearanceExtra: 0.18,
        weight: 95,
        price: 790,
        badge: 'งานเกษตร',
        chips: ['95 ก.', '฿790', 'ยกสูง +18 ซม.', 'โช้กรับแรงลงจอดหนัก'],
        desc: 'ขาตั้งกางกว้างยกตัวลำสูงพ้นถังปุ๋ยและกิมบอลใหญ่ ทนแรงกระแทกตอนลงจอดหนักได้สูงขึ้น',
        lesson: 'โดรนที่ติดถังพ่นปุ๋ยหรือกิมบอลใต้ท้องต้องใช้ขาตั้งสูงพิเศษเพื่อไม่ให้หัวฉีดหรือเลนส์กล้องกระแทกพื้นตอนลงจอด'
      },
      {
        id: 'safety_parachute',
        name: 'ชุดร่มชูชีพฉุกเฉินอัตโนมัติ',
        gearType: 'parachute',
        hasGuards: false,
        hasParachute: true,
        clearanceExtra: 0.04,
        weight: 115,
        price: 2290,
        badge: 'กางร่มอัตโนมัติ',
        chips: ['115 ก.', '฿2,290', 'กางอัตโนมัติเมื่อตก > 5 ม.', 'ลดแรงกระแทกปลอดภัย'],
        desc: 'แคปซูลร่มชูชีพบนหลังเครื่อง กางร่ม 3D ชะลอความเร็วตกอัตโนมัติหากแบตหมดหรือ Disarm กลางอากาศ!',
        lesson: 'ร่มชูชีพกู้ภัย (ASTM F3322) ช่วยลดความเร็วตกจากกว่า 15 ม./วิ ลงเหลือ ~3 ม./วิ ทำให้โดรนราคาแพงและคนด้านล่างปลอดภัยเมื่อเกิดเหตุฉุกเฉิน'
      }
    ]
  }
};

const TOTAL_PART_SLOTS = Object.keys(DRONE_PARTS).length; // 12 modular parts

/**
 * 4 Engineering Presets (Aerial / FPV / Heavy Survey / Agri Sprayer)
 */
const DRONE_PRESETS = {
  default_aerial: {
    name: 'ชุดแนะนำ 450 มม. (สมดุลครบทุกภารกิจ)',
    parts: {
      frame: 'frame_450',
      motor: 'motor_2212',
      propeller: 'prop_1045',
      esc: 'esc_30a',
      battery: 'bat_4s3300',
      fc: 'fc_f7',
      camera: 'cam_4k_gimbal',
      gps: 'gps_m8n',
      rx: 'rx_elrs',
      sensor: 'sensor_optflow',
      payload: 'payload_none',
      safety: 'safety_standard'
    }
  },
  empty_build: {
    name: `เริ่มต้นประกอบใหม่ (0/${TOTAL_PART_SLOTS})`,
    parts: {
      frame: null,
      motor: null,
      propeller: null,
      esc: null,
      battery: null,
      fc: null,
      camera: null,
      gps: null,
      rx: null,
      sensor: null,
      payload: null,
      safety: null
    }
  },
  fpv_racer: {
    name: 'FPV 250 มม. คล่องตัวสูง',
    parts: {
      frame: 'frame_250',
      motor: 'motor_2306',
      propeller: 'prop_5030',
      esc: 'esc_30a',
      battery: 'bat_3s2200',
      fc: 'fc_f4',
      camera: 'cam_light',
      gps: 'gps_none',
      rx: 'rx_elrs',
      sensor: 'sensor_none',
      payload: 'payload_none',
      safety: 'safety_guards'
    }
  },
  heavy_survey: {
    name: 'Heavy Survey 650 มม. (Thermal + LiDAR + ร่ม)',
    parts: {
      frame: 'frame_650',
      motor: 'motor_2814',
      propeller: 'prop_1255',
      esc: 'esc_45a',
      battery: 'bat_6s5200',
      fc: 'fc_h7',
      camera: 'cam_thermal',
      gps: 'gps_rtk',
      rx: 'rx_dual',
      sensor: 'sensor_lidar',
      payload: 'payload_none',
      safety: 'safety_parachute'
    }
  },
  agri_sprayer: {
    name: 'โดรนเกษตร 800 มม. (พ่นปุ๋ย + NDVI + RTK)',
    parts: {
      frame: 'frame_800_agri',
      motor: 'motor_3515',
      propeller: 'prop_1655',
      esc: 'esc_60a',
      battery: 'bat_6s10000',
      fc: 'fc_h7',
      camera: 'cam_ndvi',
      gps: 'gps_rtk',
      rx: 'rx_dual',
      sensor: 'sensor_lidar',
      payload: 'payload_sprayer',
      safety: 'safety_tall_agri'
    }
  }
};

/**
 * Compatibility checker between selected frame, motor, propeller, ESC, battery & payload
 */
function checkOptionCompatibility(partKey, opt, assembly) {
  const frameOpt = assembly.frame
    ? DRONE_PARTS.frame.options.find(o => o.id === assembly.frame)
    : null;
  const motorOpt = assembly.motor
    ? DRONE_PARTS.motor.options.find(o => o.id === assembly.motor)
    : null;
  const payloadOpt = assembly.payload
    ? DRONE_PARTS.payload.options.find(o => o.id === assembly.payload)
    : null;
  const safetyOpt = assembly.safety
    ? DRONE_PARTS.safety.options.find(o => o.id === assembly.safety)
    : null;

  if (partKey === 'motor' && frameOpt) {
    if (opt.diameter > frameOpt.maxMotorDiameter) {
      return {
        compatible: false,
        reason: `ใส่ไม่ได้ — มอเตอร์ Ø${opt.diameter} มม. ใหญ่เกินรูยึดของ${frameOpt.name} (รับสูงสุด ${frameOpt.maxMotorDiameter} มม.)`
      };
    }
  }

  if (partKey === 'propeller' && frameOpt) {
    if (opt.size > frameOpt.maxPropSize) {
      return {
        compatible: false,
        reason: `ใส่ไม่ได้ — ใบพัด ${opt.size}" ใหญ่เกิน${frameOpt.name} (รับใบสูงสุด ${Math.floor(frameOpt.maxPropSize)}")`
      };
    }
    if (opt.size < frameOpt.minPropSize) {
      return {
        compatible: false,
        reason: `ใส่ไม่ได้ — ${frameOpt.name} ต้องใช้ใบพัด ≥ ${frameOpt.minPropSize}" เพื่อให้แรงยกพอ`
      };
    }
  }

  if (partKey === 'esc' && motorOpt) {
    if (opt.rating < motorOpt.maxAmpsPerMotor) {
      return {
        compatible: false,
        reason: `ใส่ไม่ได้ — ESC ${opt.rating}A เล็กเกินไปสำหรับมอเตอร์นี้ (กินกระแสพีคสูงถึง ${motorOpt.maxAmpsPerMotor}A)`
      };
    }
  }

  if (partKey === 'payload' && frameOpt) {
    if ((opt.weight || 0) > (frameOpt.maxPayloadGrams || 1200)) {
      return {
        compatible: false,
        reason: `ใส่ไม่ได้ — ${opt.name} หนัก ${opt.weight} ก. เกินขีดจำกัดของ${frameOpt.name} (รับสูงสุด ${frameOpt.maxPayloadGrams} ก.)`
      };
    }
  }

  if (partKey === 'frame') {
    if (payloadOpt && (payloadOpt.weight || 0) > (opt.maxPayloadGrams || 1200)) {
      return {
        compatible: false,
        reason: `โครงนี้รับเพย์โหลดสูงสุด ${opt.maxPayloadGrams} ก. ไม่พอสำหรับ${payloadOpt.name} (${payloadOpt.weight} ก.)`
      };
    }
  }

  if (partKey === 'safety' && opt.gearType === 'guards' && frameOpt && frameOpt.wheelbase >= 800) {
    return {
      compatible: false,
      reason: 'โครงเกษตร 800 มม. ใบพัด 16" ใหญ่เกินการ์ดกันใบพัดมาตรฐาน — แนะนำใช้ขาตั้งสูง Heavy-Duty หรือร่มชูชีพ'
    };
  }

  if (partKey === 'payload' && (opt.payloadType === 'sprayer' || opt.payloadType === 'spreader') && safetyOpt && safetyOpt.gearType === 'guards') {
    return {
      compatible: false,
      reason: 'ถังปุ๋ยเกษตรต้องใช้คู่กับโครง 650–800 มม. และขาตั้งสูงหรือมาตรฐาน (ไม่รองรับการ์ดกันใบพัดเล็ก)'
    };
  }

  return { compatible: true, reason: '' };
}

/**
 * Missions (Free Flight, Basic Hover, Agri Survey, Wind Tower)
 */
const MISSIONS = [
  {
    id: 'free_flight',
    title: 'บินอิสระ',
    category: 'ไม่มีเป้าหมาย · ไม่จำกัดเวลา',
    desc: 'บินสำรวจพื้นที่หุบเขา ทดลองพ่นปุ๋ยบนแปลงเกษตร (กด F) ลองโหมด GPS/ATTI/RTH หรือร่มชูชีพได้อิสระ',
    buttonText: '▶ ออกบินอิสระเลย',
    windSpeed: 0.6,
    timeLimit: null,
    targetAlt: null,
    checkpoints: [],
    requirements: {
      minTW: 1.1,
      minTimeSec: 30,
      minAlt: 5,
      needGps: false,
      minPayload: 0
    }
  },
  {
    id: 'mission_basic',
    title: 'ฝึกบินพื้นฐาน',
    category: 'บทเรียน: การบังคับ',
    desc: 'ทำความรู้จักเครื่องที่ประกอบมา — บินขึ้น ลอยตัว แล้วลงจอดให้เรียบร้อย',
    windSpeed: 1.5,
    timeLimit: 180, // 03:00
    targetAlt: 8,
    buttonText: '▶ พร้อมออกบิน',
    statsPill: 'ลม 1.5 ม./วิ · เวลาจำกัด 03:00',
    checkpoints: [
      { x: 0, y: 8, z: 0, radius: 4, name: 'ลอยตัวเหนือลานจอด (ความสูง 8 ม.)' }
    ],
    requirements: {
      minTW: 1.5,
      minTimeSec: 45,
      minAlt: 8,
      needGps: false,
      minPayload: 0
    }
  },
  {
    id: 'mission_agri',
    title: 'สำรวจพื้นที่เกษตร',
    category: 'บทเรียน: เพย์โหลดและการนำทาง',
    desc: 'บินสำรวจแปลงนาทั้ง 4 โซน (หากติดถังปุ๋ยสามารถกด F พ่นปุ๋ยและลงเติมที่แท่น Refill ได้!) แล้วกลับลงจอด',
    windSpeed: 2.6,
    timeLimit: 300, // 05:00
    targetAlt: 18,
    buttonText: '▶ พร้อมออกบิน',
    statsPill: 'ลม 2.6 ม./วิ · เวลาจำกัด 05:00',
    checkpoints: [
      { x: -45, y: 15, z: -35, radius: 6, name: 'จุดสำรวจที่ 1 (แปลงนาทิศเหนือ)' },
      { x: -65, y: 18, z: 20, radius: 6, name: 'จุดสำรวจที่ 2 (แปลงนาทิศตะวันตก)' },
      { x: -20, y: 14, z: 60, radius: 6, name: 'จุดสำรวจที่ 3 (สวนผลไม้ริมน้ำ)' },
      { x: 30, y: 16, z: 40, radius: 6, name: 'จุดสำรวจที่ 4 (แปลงพืชไร่ริมเขา)' }
    ],
    requirements: {
      minTW: 1.8,
      minTimeSec: 90,
      minAlt: 18,
      needGps: true,
      minPayload: 40
    }
  },
  {
    id: 'mission_tower',
    title: 'ตรวจสอบเสาสูง',
    category: 'บทเรียน: พลังงานและความสูง',
    desc: 'บินขึ้นไปให้ถึง 45 เมตร ในลมแรง เก็บภาพและกลับลงมาให้ปลอดภัย',
    windSpeed: 5.0,
    timeLimit: 280, // 04:40
    targetAlt: 45,
    buttonText: '▶ พร้อมออกบิน',
    statsPill: 'ลม 5 ม./วิ · เวลาจำกัด 04:40',
    checkpoints: [
      { x: 0, y: 45, z: 0, radius: 6, name: 'ยอดเสาตรวจสอบ (ความสูง 45 ม.)' }
    ],
    requirements: {
      minTW: 2.0,
      minTimeSec: 70,
      minAlt: 45,
      needGps: false,
      minPayload: 100
    }
  }
];

function calculateDroneStats(assembly) {
  let totalWeight = 0;
  let totalPrice = 0;
  let installedCount = 0;
  const resolvedParts = {};

  for (const partKey in DRONE_PARTS) {
    const selectedOptionId = assembly[partKey];
    if (selectedOptionId) {
      const partDef = DRONE_PARTS[partKey];
      const opt = partDef.options.find(o => o.id === selectedOptionId);
      if (opt) {
        resolvedParts[partKey] = opt;
        totalWeight += (opt.weight || 0);
        totalPrice += (opt.price || 0);
        installedCount++;
      }
    }
  }

  const isComplete = installedCount === TOTAL_PART_SLOTS;
  const compatibilityIssues = [];

  if (resolvedParts.frame && resolvedParts.motor) {
    const c = checkOptionCompatibility('motor', resolvedParts.motor, assembly);
    if (!c.compatible) compatibilityIssues.push(c.reason);
  }
  if (resolvedParts.frame && resolvedParts.propeller) {
    const c = checkOptionCompatibility('propeller', resolvedParts.propeller, assembly);
    if (!c.compatible) compatibilityIssues.push(c.reason);
  }
  if (resolvedParts.motor && resolvedParts.esc) {
    const c = checkOptionCompatibility('esc', resolvedParts.esc, assembly);
    if (!c.compatible) compatibilityIssues.push(c.reason);
  }
  if (resolvedParts.frame && resolvedParts.payload) {
    const c = checkOptionCompatibility('payload', resolvedParts.payload, assembly);
    if (!c.compatible) compatibilityIssues.push(c.reason);
  }
  if (resolvedParts.frame && resolvedParts.safety) {
    const c = checkOptionCompatibility('safety', resolvedParts.safety, assembly);
    if (!c.compatible) compatibilityIssues.push(c.reason);
  }

  if (!isComplete) {
    return {
      isComplete: false,
      isCompatible: compatibilityIssues.length === 0,
      compatibilityIssues,
      installedCount,
      totalSlots: TOTAL_PART_SLOTS,
      totalWeight,
      totalThrust: 0,
      twRatio: 0,
      estHoverMinutes: 0,
      estHoverSecStr: '00:00',
      maxCurrentAmps: 0,
      totalPrice,
      maxClimbAltitude: 0,
      resolvedParts
    };
  }

  const motor = resolvedParts.motor;
  const prop = resolvedParts.propeller;
  const bat = resolvedParts.battery;

  const voltFactor = Math.pow(bat.voltage / 14.8, 0.85);
  // Real motor torque ceiling: small high-KV motors cannot spin oversized props at full efficiency
  const effPropMult = Math.min(prop.thrustMultiplier, motor.maxPropThrustMult || 2.0);
  const totalThrust = Math.round(motor.baseThrustPerMotor * 4 * effPropMult * voltFactor);
  const twRatio = Number((totalThrust / Math.max(1, totalWeight)).toFixed(2));
  const hoverThrottle = Math.min(1.0, Math.pow(totalWeight / Math.max(1, totalThrust), 0.50));

  // Real Rotorcraft Actuator-Disk Momentum Theory: P_hover ∝ W^(3/2) / sqrt(diskArea)
  // Doubling total mass increases hover power by 2^1.5 = 2.83x (not linear!), dramatically reducing flight time!
  const massKg = Math.max(0.22, totalWeight / 1000);
  const propSizeIn = Math.max(4, prop.size || 10);
  const propEff = prop.efficiencyFactor || 1.0;
  const packWh = (bat.voltage * bat.capacity) / 1000; // Watt-hours
  const usableWh = packWh * 0.85;
  const hoverPowerWatts = 320 * (Math.pow(massKg, 1.5) / ((propSizeIn / 10) * propEff)) + 16;
  const hoverMinutesTotal = (usableWh / Math.max(20, hoverPowerWatts)) * 60;
  const hoverMins = Math.floor(hoverMinutesTotal);
  const hoverSecs = Math.floor((hoverMinutesTotal - hoverMins) * 60);
  const estHoverSecStr = `${String(hoverMins).padStart(2, '0')}:${String(hoverSecs).padStart(2, '0')}`;
  const hoverCurrentAmps = Number(((bat.capacity / 1000) * 0.85 * 60 / Math.max(0.8, hoverMinutesTotal)).toFixed(1));
  const maxClimbAltitude = twRatio <= 1.0
    ? 0
    : Math.min(220, Math.round(Math.max(0, twRatio - 1.0) * 34 + (packWh * 0.85)));

  return {
    isComplete: true,
    isCompatible: compatibilityIssues.length === 0,
    compatibilityIssues,
    installedCount,
    totalSlots: TOTAL_PART_SLOTS,
    totalWeight,
    totalThrust,
    twRatio,
    hoverThrottle,
    estHoverMinutes: hoverMinutesTotal,
    estHoverSecStr,
    maxCurrentAmps: hoverCurrentAmps,
    totalPrice,
    maxClimbAltitude,
    resolvedParts
  };
}

function checkMissionReadiness(stats, mission) {
  const total = stats.totalSlots || TOTAL_PART_SLOTS;
  if (!stats.isComplete) {
    return {
      ready: false,
      checks: [
        { label: `ต้องประกอบให้ครบ ${total} ชิ้นก่อน (ขาด ${total - stats.installedCount} ชิ้น)`, pass: false }
      ]
    };
  }

  const checks = [];
  const req = mission.requirements;
  const p = stats.resolvedParts;

  if (!stats.isCompatible) {
    checks.push({ label: 'ชิ้นส่วนต้องเข้ากันได้ทั้งหมด', pass: false });
  }

  const twPass = stats.twRatio >= req.minTW;
  checks.push({
    label: `แรงยกพอ (T/W ${stats.twRatio})`,
    pass: twPass
  });

  if (req.minPayload > 0) {
    const payloadWeight = (p.camera && p.camera.weight) || 0;
    checks.push({
      label: `เพย์โหลดกล้องพอ (${payloadWeight} ก. ≥ ${req.minPayload} ก.)`,
      pass: payloadWeight >= req.minPayload
    });
  }

  if (req.needGps) {
    const hasGps = p.gps && p.gps.hasGps;
    checks.push({
      label: 'มี GPS สำหรับนำทาง',
      pass: Boolean(hasGps)
    });
  }

  const totalHoverSecs = stats.estHoverMinutes * 60;
  const reqMinStr = `${String(Math.floor(req.minTimeSec / 60)).padStart(2, '0')}:${String(req.minTimeSec % 60).padStart(2, '0')}`;
  checks.push({
    label: `เวลาบินพอ (${stats.estHoverSecStr} ≥ ${reqMinStr})`,
    pass: totalHoverSecs >= req.minTimeSec
  });

  checks.push({
    label: `ไต่ถึง ${req.minAlt} ม. ได้`,
    pass: stats.maxClimbAltitude >= req.minAlt
  });

  return {
    ready: checks.every(c => c.pass),
    checks
  };
}

/**
 * Dynamic Thailand Drone Law Compliance Classifier (NBTC / กสทช. + CAAT / กพท.)
 * Classifies the user's exact assembled drone by weight, camera, and agricultural payload!
 */
function evaluateThailandDroneLaw(stats) {
  const p = (stats && stats.resolvedParts) || {};
  const weightG = (stats && stats.totalWeight) || 0;
  const weightKg = weightG / 1000;
  const hasCamera = Boolean(p.camera && p.camera.type && p.camera.type !== 'none');
  const payloadType = (p.payload && p.payload.payloadType) || 'none';
  const isAgri = payloadType === 'sprayer' || payloadType === 'spreader';
  const isHeavyClass = weightKg > 2.0; // > 2 kg enters 2–25 kg CAAT bracket
  const needCAAT = hasCamera || isHeavyClass || isAgri;
  const needNBTC = true; // Radio-controlled aircraft / RF transmitter registration within 30 days
  const minHorizDistanceM = isHeavyClass ? 50 : 30;
  const minPilotAge = isHeavyClass ? 20 : 18;

  let weightClassLabel = 'พิกัด ≤ 2 กก. (ขนาดเล็ก/กลาง)';
  if (weightKg > 25) weightClassLabel = 'พิกัด > 25 กก. (ต้องขออนุญาต รมว.คมนาคม)';
  else if (isHeavyClass) weightClassLabel = 'พิกัด 2 – 25 กก. (ขนาดใหญ่/งานอาชีพ)';

  const reasonParts = [];
  if (hasCamera) reasonParts.push('ติดตั้งกล้องบันทึกภาพ');
  if (isHeavyClass) reasonParts.push(`น้ำหนัก ${weightKg.toFixed(2)} กก. (> 2 กก.)`);
  if (isAgri) reasonParts.push('มีอุปกรณ์พ่น/หว่านเกษตร');

  const summaryShort = needCAAT
    ? `ต้องขึ้นทะเบียน กสทช. + กพท. (${reasonParts.join(' + ')}) · ห่างคน/อาคาร ≥ ${minHorizDistanceM} ม. · สูง ≤ 90 ม.`
    : `ไม่มีกล้องและน้ำหนัก ≤ 2 กก. (กพท. ยกเว้นสันทนาการ แต่ต้องขึ้นทะเบียน กสทช. · ห่างคน ≥ 30 ม. · สูง ≤ 90 ม.)`;

  const requirements = [
    {
      agency: 'กสทช. (NBTC · AnyRegis)',
      required: needNBTC,
      detail: 'ขึ้นทะเบียนครอบครองเครื่องวิทยุคมนาคมภายใน 30 วัน (โทษไม่ทำ: ปรับสูงสุด 100,000 ฿ / คุก 5 ปี)'
    },
    {
      agency: 'กพท. (CAAT · UAS Portal)',
      required: needCAAT,
      detail: needCAAT
        ? `ต้องขึ้นทะเบียนผู้บังคับและอากาศยาน (อายุใบอนุญาต 2 ปี · ผู้บังคับอายุ ≥ ${minPilotAge} ปี) + ประกันภัยบุคคลภายนอก ≥ 1,000,000 ฿`
        : 'ยกเว้นการขึ้นทะเบียนผู้บังคับเพื่อสันทนาการ (เพราะไม่มีกล้องและหนัก ≤ 2 กก.) แต่ต้องทำตามกฎการบินทุกข้อ'
    },
    {
      agency: 'กฎการบินและระยะปลอดภัย',
      required: true,
      detail: `เพดานบินสูงสุด ≤ 90 ม. · ห่างสนามบิน ≥ 9 กม. · ห่างคน/ยานพาหนะ/อาคาร ≥ ${minHorizDistanceM} ม. · บินกลางวันแบบเห็นด้วยตาเปล่า (VLOS)`
    }
  ];

  if (isAgri) {
    requirements.push({
      agency: 'ข้อบังคับโดรนเกษตร (พ่นปุ๋ย/หว่านเม็ด)',
      required: true,
      detail: 'ห้ามปล่อยละอองปุ๋ยปลิวเข้าชุมชน/แหล่งน้ำ · ตรวจลมจากถุงลมก่อนบิน · หากรุ่นจริงน้ำหนักรวม > 25 กก. ต้องอบรมหลักสูตรรับรองและขออนุญาต รมว.คมนาคม'
    });
  }

  return {
    weightKg: Number(weightKg.toFixed(2)),
    weightClassLabel,
    hasCamera,
    isAgri,
    isHeavyClass,
    needNBTC,
    needCAAT,
    minHorizDistanceM,
    minPilotAge,
    maxAltitudeM: 90,
    minAirportKm: 9,
    summaryShort,
    requirements
  };
}

function generateDebrief(flightLog, stats) {
  const timeStr = flightLog.timeStr || '01:30';
  const batteryUsedPercent = Math.min(100, Math.round(100 - flightLog.batteryRemainPercent));
  const maxAlt = Math.round(flightLog.maxAltitude || 12);
  const tw = stats.twRatio || 4.1;
  const p = stats.resolvedParts || {};
  const law = evaluateThailandDroneLaw(stats);

  let advice = `บินได้ ${timeStr} ไต่สูงสุด ${maxAlt} ม. ใช้แบตไป ${batteryUsedPercent}% ชุดนี้ T/W ${tw} — `;
  if (p.payload && (p.payload.payloadType === 'sprayer' || p.payload.payloadType === 'spreader')) {
    advice += 'สังเกตว่าเมื่อพ่น/หว่านปุ๋ยออกไป น้ำหนักรวมจะเบาลงแบบเรียลไทม์ทำให้กินกระแสน้อยลงและควบคุมคล่องขึ้นเหมือนโดรนเกษตรจริง!';
  } else if (p.battery && p.battery.chemistry === 'liion') {
    advice += 'แบต Li-Ion ช่วยยืดเวลาบินได้นานกว่า LiPo แต่สังเกตว่าเวลาไต่เต็มกำลังแรงดันไฟจะตกเร็วกว่า (C-Rating ต่ำกว่า)';
  } else if (tw >= 4.0) {
    advice += 'ยิ่งค่านี้สูง เครื่องจะตอบสนองไวและไต่เร็ว แต่กินไฟมากขึ้น สังเกตว่าไต่ค้างนาน ๆ กินไฟกว่าบินราบมาก';
  } else if (tw >= 2.3) {
    advice += 'อัตราส่วนแรงยกต่อน้ำหนักสมดุลดีมาก ควบคุมได้นิ่ง เหมาะกับการบินสำรวจหรือบรรทุกเพย์โหลดที่ต้องการความเสถียร';
  } else if (tw <= 1.02) {
    advice += 'น้ำหนักรวมเกินแรงยกสูงสุดของมอเตอร์ (T/W ≤ 1.0) ทำให้ยกตัวไม่ขึ้นในโลกจริง! ลองลดขนาดแบตเตอรี่/ถอดเพย์โหลด หรืออัปเกรดมอเตอร์และใบพัดให้ใหญ่ขึ้น';
  } else if (tw < 1.6) {
    advice += 'โดรนค่อนข้างหนัก (T/W ต่ำกว่า 1.6) สังเกตว่าไต่ช้า กินไฟสูง และระยะเบรกยาว — ควรลดน้ำหนักหรือเพิ่มแรงยกเพื่อให้บินปลอดภัยขึ้น';
  } else {
    advice += 'อัตราส่วน T/W ค่อนข้างต่ำ เครื่องตอบสนองช้า ควรลดน้ำหนักเพย์โหลดหรือเพิ่มขนาดมอเตอร์และใบพัด';
  }

  // Append Thailand Drone Law compliance debrief!
  if (maxAlt > 90 || flightLog.lawAltitudeViolated) {
    advice += ` | ⚖️ เตือนกฎหมายไทย (กพท.): เที่ยวบินนี้ไต่สูง ${maxAlt} ม. เกินเพดานกฎหมาย 90 ม.! ในชีวิตจริงมีโทษปรับสูงสุด 40,000 บาท หรือจำคุกไม่เกิน 1 ปี`;
  } else {
    advice += ` | ⚖️ ผ่านเกณฑ์กฎหมายไทย (กพท.): รักษาเพดานบิน ≤ 90 ม. เรียบร้อย (พิกัดชุดนี้ห่างคน/อาคารจริง ≥ ${law.minHorizDistanceM} ม.)`;
  }

  return {
    timeStr,
    batteryRemainPercent: Math.max(0, Math.round(flightLog.batteryRemainPercent)),
    maxAlt,
    advice
  };
}

if (typeof window !== 'undefined') {
  window.DRONE_PARTS = DRONE_PARTS;
  window.TOTAL_PART_SLOTS = TOTAL_PART_SLOTS;
  window.DRONE_PRESETS = DRONE_PRESETS;
  window.MISSIONS = MISSIONS;
  window.checkOptionCompatibility = checkOptionCompatibility;
  window.calculateDroneStats = calculateDroneStats;
  window.checkMissionReadiness = checkMissionReadiness;
  window.evaluateThailandDroneLaw = evaluateThailandDroneLaw;
  window.generateDebrief = generateDebrief;
}
