//
//  Utils.swift
//  GrootPlayer
//
//  Created by Kyungjin Lee on 2020/07/15.
//  Copyright © 2020 Kyungjin Lee. All rights reserved.
//

import Foundation
import Metal
import MetalKit


public extension FileManager {
  static var documentsDirectoryURL: URL {
    `default`.urls(for: .documentDirectory, in: .userDomainMask)[0]
  }
}


public struct Queue<T> {
  fileprivate var array = [T]()

  public var isEmpty: Bool {
    return array.isEmpty
  }
  
  public var count: Int {
    return array.count
  }

  public mutating func enqueue(_ element: T) {
    array.append(element)
  }
  
  public mutating func dequeue() -> T? {
    if isEmpty {
      return nil
    } else {
      return array.removeFirst()
    }
  }
  
  public var front: T? {
    return array.first
  }
}
