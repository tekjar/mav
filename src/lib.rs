#[macro_use]
extern crate quote;
extern crate serde;
#[macro_use]
extern crate serde_derive;
extern crate serde_json;
extern crate syn;

pub mod mavlink;

use std::fs::File;
use std::io::BufReader;
use std::io::Write;

use serde_json::{Error, Value};
use syn::Ident;

#[derive(Debug, Serialize, Deserialize)]
struct MavLink {
    version: String,
}

#[derive(Debug, Serialize, Deserialize)]
struct Mav {
    mavlink: MavLink,
}

pub fn generate() {
    let schema_file = File::open("common.json").unwrap();
    let schema: Value = serde_json::from_reader(schema_file).unwrap();

    let enums = &schema["mavlink"]["enums"]["enum"][0]["entry"];
    let enums = enums.as_array().unwrap();

    for i in enums.iter() {
        println!("{:?}", i["_name"]);
    }
}

pub fn try() {
    let mut mavlink = File::create("src/mavlink.rs").unwrap();

    let names = vec![
        Ident::from("Altitude"),
        Ident::from("Gps"),
        Ident::from("Gyro"),
    ];

    let kinds = vec!["u8", "u8", "u8"];

    let tokens = quote!(
        # ( struct # names {
            value: #kinds
        })*
    );

    mavlink.write_all(tokens.to_string().as_bytes());
}
