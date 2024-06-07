
#[allow(dead_code)]
#[allow(non_snake_case)]
pub fn Fresh<T>(_var:T) -> (){}

#[allow(dead_code)]
#[allow(non_snake_case)]
pub fn Consistent<T>(_var:T, _id:u16) -> (){}

#[allow(dead_code)]
#[allow(non_snake_case)]
fn FreshConsistent<T>(_var:T, _id:u16) -> (){}


#[macro_export]
macro_rules! nv {
    ($name:ident : $ty:ty = $expr:expr) => {
	unsafe {
	    #[link_section = ".fram_section"]
	    static mut $name: Option<$ty> = None;

	    let used = $name.is_some();
	    if used {
		None
	    } else {
		$name = Some($expr);
		$name.as_mut()

	    }
	}
    };
}

#[macro_export]
macro_rules! big_nv {
    ($name:ident : $ty:ty = $expr:expr) => {
	unsafe {
	    #[link_section = ".fram_section"]
	    static mut $name:$ty = $expr;
		& mut $name

	    }
    };
}