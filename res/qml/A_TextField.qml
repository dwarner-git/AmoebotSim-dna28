/* Copyright (C) 2021 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

import QtQuick 2.0
import QtQuick.Controls 1.1
import QtQuick.Controls.Styles 1.1

TextField {
  style: TextFieldStyle {
    background: Rectangle {
      implicitWidth: 260
      implicitHeight: 25
      border.width: 1
      border.color: "#888"
      radius: 4
      color: "#eee"
      opacity: 0.9
    }
  }
}
